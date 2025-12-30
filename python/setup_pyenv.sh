# 1. Install pyenv dependencies
sudo apt update
sudo apt install -y make build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev \
libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python3-openssl git

# 2. Install pyenv
curl https://pyenv.run | bash

# 3. Add to ~/.bashrc (copy the output from above)
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc

# 4. Restart terminal or source bashrc
exec "$SHELL"

# 5. Install Python 3.12
pyenv install 3.12.7
cd ~/2026_dev/2D-Camera-SLAM/python
pyenv local 3.12.7  # sets .python-version file

# 6. Create venv and install rerun
python -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install rerun-sdk numpy
