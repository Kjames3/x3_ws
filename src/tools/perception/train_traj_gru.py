#!/usr/bin/env python3
"""Train a GRU pedestrian trajectory predictor on trajectory_baselines --export windows.

Input: windows.npz (hist [N, H, 4: x y vx vy] odom frame, fut [N, F, 2], run [N]).
Every window is made translation-invariant (relative to the last history
position) and augmented with random rotation + mirror, since the odom frame's
orientation means nothing. The GRU predicts a RESIDUAL on top of cv_est (the
estimator velocity extrapolated straight), so with no signal it falls back to
the baseline the harness found best.

Evaluation is grouped by run (K folds, no run in both train and test), and
each fold reports ADE/FDE for cv_est and the GRU on the same test windows.

Progress lines "epoch <fold>/<k> <e>/<E> ..." are written to --log so the
job-progress page can count them (step pattern '^epoch ').

  python3 train_traj_gru.py windows.npz --out out_dir --log train.log
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HORIZONS = (0.5, 1.0, 2.0, 3.0)


class TrajGRU(nn.Module):
    def __init__(self, fut_n, hidden=64, layers=2, dropout=0.1):
        super().__init__()
        self.fut_n = fut_n
        self.inp = nn.Linear(6, hidden)
        self.gru = nn.GRU(hidden, hidden, layers, batch_first=True, dropout=dropout if layers > 1 else 0)
        self.head = nn.Sequential(nn.Linear(hidden, hidden), nn.ReLU(), nn.Linear(hidden, fut_n * 2))

    def forward(self, x):                       # x: [B, H, 6]
        h, _ = self.gru(torch.relu(self.inp(x)))
        return self.head(h[:, -1]).view(-1, self.fut_n, 2)


def features(hist, dt):
    """[N, H, 6]: position rel. to last, estimator velocity, finite-difference velocity."""
    rel = hist[:, :, :2] - hist[:, -1:, :2]
    fd = np.diff(hist[:, :, :2], axis=1, prepend=hist[:, :1, :2]) / dt
    return np.concatenate([rel, hist[:, :, 2:4], fd], 2).astype(np.float32)


def cv_est(hist, fut_n, dt):
    steps = (np.arange(1, fut_n + 1) * dt)[None, :, None]
    return (hist[:, -1:, 2:4] * steps).astype(np.float32)          # relative to p0


def augment(x, y, b):
    """Same random rotation (+ mirror) on every 2-vector channel."""
    B = x.shape[0]
    th = torch.rand(B, device=x.device) * 2 * math.pi
    c, s = torch.cos(th), torch.sin(th)
    R = torch.stack([torch.stack([c, -s], -1), torch.stack([s, c], -1)], -2)  # [B,2,2]
    flip = (torch.rand(B, device=x.device) < 0.5).float() * -2 + 1
    R = R * torch.stack([torch.ones_like(flip), flip], -1)[:, None, :]
    rot = lambda v: torch.einsum('bij,btj->bti', R, v)
    x = torch.cat([rot(x[..., 0:2]), rot(x[..., 2:4]), rot(x[..., 4:6])], -1)
    return x, rot(y), rot(b)


def metrics(pred, gt, rate):
    e = np.linalg.norm(pred - gt, axis=2)
    out = {}
    for hz in HORIZONS:
        k = int(round(hz * rate))
        if k <= e.shape[1]:
            out[str(hz)] = dict(ade=float(e[:, :k].mean()), fde=float(e[:, k - 1].mean()))
    return out


def log(f, msg):
    print(msg, flush=True)
    if f:
        f.write(msg + '\n')
        f.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--out', default='traj_gru_out')
    ap.add_argument('--log')
    ap.add_argument('--folds', type=int, default=5)
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--hidden', type=int, default=64)
    ap.add_argument('--lr', type=float, default=2e-3)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()

    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    d = np.load(a.npz)
    hist, fut, run, rate = d['hist'], d['fut'], d['run'], float(d['rate'])
    runs = list(d['runs'])
    dt, fut_n = 1.0 / rate, fut.shape[1]
    X = features(hist, dt)
    Y = (fut - hist[:, -1:, :2]).astype(np.float32)
    Bse = cv_est(hist, fut_n, dt)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    lf = open(a.log, 'w') if a.log else None
    log(lf, 'device %s  windows %d  runs %d  folds %d  epochs %d' % (dev, len(X), len(runs), a.folds, a.epochs))

    # folds over runs, balanced by window count
    order = rng.permutation(len(runs))
    counts = np.bincount(run, minlength=len(runs))
    fold_of, load = np.zeros(len(runs), int), np.zeros(a.folds)
    for r in sorted(order, key=lambda r: -counts[r]):
        f = int(np.argmin(load))
        fold_of[r], load[f] = f, load[f] + counts[r]

    results, P_all, B_all, G_all = [], [], [], []
    t_start = time.time()
    for k in range(a.folds):
        te = fold_of[run] == k
        tr = ~te
        # 10% of the training runs held out for early stopping
        tr_runs = [r for r in range(len(runs)) if fold_of[r] != k and counts[r]]
        val_runs = set(rng.choice(tr_runs, max(1, len(tr_runs) // 10), replace=False).tolist())
        va = tr & np.isin(run, list(val_runs))
        tr = tr & ~va
        T = lambda m: [torch.tensor(v[m], device=dev) for v in (X, Y, Bse)]
        xtr, ytr, btr = T(tr)
        xva, yva, bva = T(va)
        xte, yte, bte = T(te)
        sx = xtr.reshape(-1, 6).std(0) + 1e-3      # rotation-symmetric enough for a scale
        model = TrajGRU(fut_n, a.hidden).to(dev)
        opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
        best, best_state = float('inf'), None
        for ep in range(a.epochs):
            model.train()
            perm = torch.randperm(len(xtr), device=dev)
            tot = 0.0
            for i in range(0, len(perm), a.batch):
                j = perm[i:i + a.batch]
                x, y, b = augment(xtr[j], ytr[j], btr[j])
                pred = b + model(x / sx)
                loss = torch.linalg.norm(pred - y, dim=-1).mean()   # ADE
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                opt.step()
                tot += loss.item() * len(j)
            sched.step()
            model.eval()
            with torch.no_grad():
                v = torch.linalg.norm(bva + model(xva / sx) - yva, dim=-1).mean().item() if len(xva) else tot
            if v < best:
                best, best_state = v, {kk: vv.clone() for kk, vv in model.state_dict().items()}
            log(lf, 'epoch %d/%d %d/%d train_ade %.3f val_ade %.3f best %.3f  %.0fs'
                % (k + 1, a.folds, ep + 1, a.epochs, tot / max(1, len(xtr)), v, best, time.time() - t_start))
        model.load_state_dict(best_state)
        model.eval()
        with torch.no_grad():
            p = (bte + model(xte / sx)).cpu().numpy()
        g, bb = yte.cpu().numpy(), bte.cpu().numpy()
        r = dict(fold=k, test_runs=[runs[i] for i in range(len(runs)) if fold_of[i] == k],
                 n_test=int(te.sum()), cv_est=metrics(bb, g, rate), gru=metrics(p, g, rate))
        results.append(r)
        P_all.append(p); B_all.append(bb); G_all.append(g)
        log(lf, 'fold %d  n=%d  FDE@1s cv %.3f gru %.3f   FDE@2s cv %.3f gru %.3f' % (
            k + 1, r['n_test'], r['cv_est']['1.0']['fde'], r['gru']['1.0']['fde'],
            r['cv_est']['2.0']['fde'], r['gru']['2.0']['fde']))
        torch.save(dict(state=best_state, scale=sx.cpu(), fut_n=fut_n, hidden=a.hidden, rate=rate),
                   out / ('gru_fold%d.pt' % k))

    P, B, G = map(np.concatenate, (P_all, B_all, G_all))
    pooled = dict(cv_est=metrics(B, G, rate), gru=metrics(P, G, rate))
    lines = ['pooled over %d held-out windows (grouped-by-run %d folds)' % (len(G), a.folds),
             '  horizon   cv_est ADE/FDE    gru ADE/FDE    FDE change']
    for hz in pooled['gru']:
        c, g = pooled['cv_est'][hz], pooled['gru'][hz]
        lines.append('  %5s s   %5.3f / %5.3f    %5.3f / %5.3f    %+5.1f%%' % (
            hz, c['ade'], c['fde'], g['ade'], g['fde'], 100 * (g['fde'] / c['fde'] - 1)))
    txt = '\n'.join(lines)
    log(lf, txt)
    (out / 'report.txt').write_text(txt + '\n')
    (out / 'results.json').write_text(json.dumps(dict(args=vars(a), folds=results, pooled=pooled), indent=1))
    log(lf, 'TRAINING DONE')


if __name__ == '__main__':
    main()
