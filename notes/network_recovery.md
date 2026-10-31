# Robot remote-access recovery (2026-10-30)

Installed on the Orin: `scripts/x3_network_watchdog.py` at
`/usr/local/sbin/x3_network_watchdog.py`, the matching service/timer under
`deploy/systemd/`, and `tailscaled.service.d/20-x3-network-ready.conf`.

Tailscale startup now waits for an IPv4 default route and successful control-plane
DNS lookup, with a bounded wait (approximately one minute). Timeout lets Tailscale
proceed with its own recovery. The current connection was not restarted; this
startup path still needs verification at the next planned boot.

The timer starts three minutes after boot and checks every minute. Three
consecutive unhealthy checks (backend not Running or Self not Online) qualify
for recovery only when DNS/default route and a verified HTTPS request to the
Tailscale control-plane key endpoint work. It logs routes, resolver status,
Tailscale status and recent daemon logs before restarting only tailscaled.
Cooldown is 15 minutes using monotonic time and /run state (reset at boot).
It never reconnects Wi-Fi or restarts robot services. It does not detect every
possible tunnel data-plane failure while Tailscale reports healthy.

Inspect:
```
systemctl status x3_network_watchdog.timer
journalctl -u x3_network_watchdog.service
python3 /usr/local/sbin/x3_network_watchdog.py --check-only
```

Disable/rollback (sudo): stop and disable `x3_network_watchdog.timer`, remove
`/etc/systemd/system/tailscaled.service.d/20-x3-network-ready.conf`, then
`systemctl daemon-reload`. No immediate Tailscale restart is needed.

Evidence: Wi-Fi acquired an address at 17:15:21; Tailscale logged DNS failures
at startup and connected at 17:21:29, before Ethernet unplug at 17:24:07.
A clock synchronization at 17:18:03 means wall-clock durations are unreliable.
The Livox profile has no gateway and never-default=yes; reconnecting it did not
break access. Wi-Fi power saving was already disabled. DNS currently works via
systemd-resolved with campus DHCP DNS. Its resolvconf-managed file points to
127.0.0.53; no resolver changes were justified by the live evidence.

Validated Python syntax, threshold/cooldown boundaries, systemd unit validation,
and live check-only/readiness paths. First installed timer run succeeded with
zero failures. GUI HTTP remained 200 and server/Tailscale remained active.
A forced outage/restart test was deliberately not run during live robot use.

## Livox clock-jump failure found during reboot verification

The Jetson started near the Unix epoch and corrected its clock after the ROS
stack had started. Livox `LidarPubHandler` used high_resolution_clock (wall clock
on this platform), incrementing last_pub_time_ by 100 ms each publication. A
large forward clock correction therefore caused continuous catch-up publication
(~1970 messages/s observed), instead of 10 Hz; GUI scans disappeared.

Applied `deploy/patches/livox-monotonic-publish.patch` to both laptop and robot
`~/ws_livox/src/livox_ros_driver2`: publication cadence now uses steady_clock
and resets the deadline to now after each interval. Sensor epoch timestamp
creation remains unchanged. Robot package rebuilt successfully and x3_server
restarted. /scan returned ~10 Hz; an actual Foxglove /scan subscription delivered
120 messages in 12 seconds without restarting Foxglove. Another cold boot is
needed to verify the clock-jump prevention itself. External driver changes are
uncommitted; preserve/reapply the patch when replacing that checkout.
