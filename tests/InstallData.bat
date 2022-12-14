@echo off
:: Download a AR device capture
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
powershell Expand-Archive loopDesktopA.zip -DestinationPath .\data -F
del loopDesktopA.zip

:: Install required external modules
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug