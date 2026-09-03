Place the live-contract model artifacts here (see ../README.md):

  model.onnx              <- vantare_bicep_curl_v1_1.onnx
  model_contract.json     <- model_contract_v1_1.json
  feature_names.json      <- feature_names_v1_1.json

These are produced by running Vantare_Bicep_Curl_Training_ONNX_v1_1.ipynb
(design Section 11.3). The 50 Hz V1 artifacts in host/notebooks/ will not
pass the contract gate — the app refuses them by design.
