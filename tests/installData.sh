# Download AR device capture
curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data
rm loopDesktopA.zip

:: Download yolact and fcn models
curl https://artifact.b-com.com/solar-generic-local/learnedModels/FCN/fcn_resnet50.onnx -L -o .\data\fcn_resnet50.onnx
curl https://artifact.b-com.com/solar-generic-local/learnedModels/yolact/yolact.onnx -L -o .\data\yolact.onnx

# Install required packagedependencies.txt
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

