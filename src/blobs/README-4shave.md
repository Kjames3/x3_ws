# 4-SHAVE blob variants

`yolo26n-pose/yolo26n-pose_4shave.blob` and `yolo26n-seg/yolo26n-seg_4shave.blob`
were built 2026-10-01 from `models/onnx/yolo26n-{pose,seg}-640x480.onnx`
(sha256 `be5d903d…`, `6fb280d7…`) with `ghcr.io/luxonis/modelconverter-rvc2:latest`
(OpenVINO 2022.3.0), locally in Docker:

```
mo --input_model <m>-640x480.onnx --output output0[,output1] --compress_to_fp16 \
   --input "images[1 3 640 480]{f32}" --mean_values "images[0.0,0.0,0.0]" \
   --scale_values "images[255.0,255.0,255.0]" --reverse_input_channels
compile_tool -d MYRIAD -ip U8 -m <m>.xml -o <m>_4shave.blob -c myriad4.conf
# myriad4.conf: MYRIAD_NUMBER_OF_SHAVES 4, MYRIAD_NUMBER_OF_CMX_SLICES 4,
#               MYRIAD_THROUGHPUT_STREAMS 1, MYRIAD_ENABLE_MX_BOOT NO
```

The OpenVINO IR reproduces the ONNX output on `.cache/pose-lite/sample-person.jpg`
(mean abs diff 0.05 pose, 0.003 seg boxes, 0.001 prototypes; FP16 rounding).
The default 8-shave blobs came from Luxonis's online converter (OpenVINO 2022.1)
and are not byte-reproducible here. Select with `scripts/oak_blob_shaves.sh 4`
(`X3_OAK_BLOB_SHAVES`). Same input, outputs and `config.json` as the 8-shave blob.
Not yet run on the camera when this was written.
