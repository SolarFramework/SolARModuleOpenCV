@echo off
:: Download a AR device capture
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
powershell Expand-Archive loopDesktopA.zip -DestinationPath .\data -F
del loopDesktopA.zip

:: Download yolact and fcn models
curl https://repository.solarframework.org/generic/LearnedModels/FCN/fcn_resnet50.onnx -L -o .\data\fcn_resnet50.onnx
curl https://repository.solarframework.org/generic/LearnedModels/yolact/yolact.onnx -L -o .\data\yolact.onnx

:: Download PSPNet and DeepLabV3 models 
echo Download PSPNet model 
curl https://repository.solarframework.org/generic/LearnedModels/PSPNet.zip -L -o PSPNet.zip
powershell Expand-Archive PSPNet.zip -DestinationPath .\data -F
del PSPNet.zip
echo Download DeepLabV3 model 
curl https://repository.solarframework.org/generic/LearnedModels/DeepLabV3.zip -L -o DeepLabV3.zip
powershell Expand-Archive DeepLabV3.zip -DestinationPath .\data -F
del DeepLabV3.zip

:: Install required external modules
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug