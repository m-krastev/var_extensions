echo Removing the existing build and install directories...
rmdir /S /Q build
rmdir /S /Q install

set ROS_DOMAIN_ID=5
echo Building the project...
colcon build

echo Setting up the environment...
install\setup.bat

echo Modifying Python shebangs...
for %f in (install\curling_pipeline\lib\curling_pipeline\*) do (
    powershell -Command "(Get-Content %f) -replace '#!/usr/bin/python3', '#!/usr/bin/env python3' | Set-Content %f"
)

echo Build and setup complete.