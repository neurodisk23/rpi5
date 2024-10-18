# rpi5

## Installing the package 

### One might face this issue :

$sudo pip3 install opencv-contrib-python
error: externally-managed-environment

× This environment is externally managed
╰─> To install Python packages system-wide, try apt install
    python3-xyz, where xyz is the package you are trying to
    install.
    
    If you wish to install a non-Debian-packaged Python package,
    create a virtual environment using python3 -m venv path/to/venv.
    Then use path/to/venv/bin/python and path/to/venv/bin/pip. Make
    sure you have python3-full installed.
    
    For more information visit http://rptl.io/venv

note: If you believe this is a mistake, please contact your Python installation or OS distribution provider. You can override this, at the risk of breaking your Python installation or OS, by passing --break-system-packages.
hint: See PEP 668 for the detailed specification.

So by this warning the rpi is telling to not to mix the pip3/pip and apt installatins as this might led to breaking of the system packages 
if you want to still install you can do : pip install xyz --break-system-packages   taking reference of  : https://stackoverflow.com/questions/75602063/pip-install-r-requirements-txt-is-failing-this-environment-is-externally-mana


TO get out of this create a virtual environment 

To check for virtual environment : sudo dpkg -l | grep venv
To make a environment : 
```
python3 -m venv <name>
to check for the pckage : cd ~/python
python3 -m venv rpitips
ls
```

source <env_name>/bin/activate
