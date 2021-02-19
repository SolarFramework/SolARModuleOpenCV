# Download AR device capture
curl https://artifact.b-com.com/solar-generic-local/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data
rm loopDesktopA.zip

# Install required packagedependencies.txt
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

