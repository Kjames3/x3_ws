import os
import urllib.request

# YOLO26 weights are published under the v8.4.0 tag of ultralytics/assets.
# Older tags (v8.2.0, v8.3.0) predate the model and return 404.
ASSETS_RELEASE = "v8.4.0"


def download_yolo_model():
    model_name = "yolo26n-pose.pt"
    url = f"https://github.com/ultralytics/assets/releases/download/{ASSETS_RELEASE}/{model_name}"

    # Determine the directory relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(script_dir, '..', 'models')
    os.makedirs(models_dir, exist_ok=True)

    target_path = os.path.join(models_dir, model_name)

    print(f"Downloading {model_name} to {target_path}...")

    try:
        # Use urllib to download the weights from GitHub releases
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req) as response, open(target_path, 'wb') as out_file:
            data = response.read()
            out_file.write(data)
        print("Download complete!")
    except Exception as e:
        print(f"Error downloading model via URL: {e}")

        # Fallback to using the ultralytics python package if installed.
        # attempt_download_asset() resolves bare model names against the
        # assets repo; download() would treat the name as a URL.
        print("Attempting to use ultralytics package to download...")
        try:
            from ultralytics.utils.downloads import attempt_download_asset
            attempt_download_asset(target_path, release=ASSETS_RELEASE)
            print("Download complete via ultralytics package!")
        except ImportError:
            print("Failed. Please install the 'ultralytics' package via pip.")


if __name__ == "__main__":
    download_yolo_model()
