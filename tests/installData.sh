# Download AR device capture
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data
rm loopDesktopA.zip

:: Download yolact and fcn models
curl https://repository.solarframework.org/generic/learnedModels/FCN/fcn_resnet50.onnx -L -o .\data\fcn_resnet50.onnx
curl https://repository.solarframework.org/generic/learnedModels/yolact/yolact.onnx -L -o .\data\yolact.onnx

:: Download PSPNet and DeepLabV3 models 
curl https://repository.solarframework.org/generic/LearnedModels/PSPNet.zip -L -o PSPNet.zip
unzip -o PSPNet.zip -d ./data
rm PSPNet.zip 
curl https://repository.solarframework.org/generic/LearnedModels/DeepLabV3.zip -L -o DeepLabV3.zip
unzip -o DeepLabV3.zip -d ./data
rm DeepLabV3.zip 

# Install required packagedependencies.txt
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

