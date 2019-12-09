#!/bin/bash
echo "[Installing dependant packages]"
sudo apt install espeak
sudo apt install python-pip
echo "[Installing python modules]"
pip install psutil
pip install pyttsx
echo "[Setting environment variables]"
cd ~
sh -c "echo \"export CR_MODEL=MUTANT\" >> ~/.bashrc"
echo "[Done!]"