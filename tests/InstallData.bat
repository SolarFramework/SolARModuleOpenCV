@echo off
:: Download a AR device capture
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
powershell Expand-Archive loopDesktopA.zip -DestinationPath .\data -F
del loopDesktopA.zip

:: Download yolact and fcn models
curl https://repository.solarframework.org/generic/learnedModels/FCN/fcn_resnet50.onnx -L -o .\data\fcn_resnet50.onnx
curl https://repository.solarframework.org/generic/learnedModels/yolact/yolact.onnx -L -o .\data\yolact.onnx

:: Install required external modules
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug