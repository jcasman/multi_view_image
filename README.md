# Multi View
This project started since 2019, Shuhei Kozasa's master thesis.
Naoki Sato [posted in the RICOH THETA developer community](https://community.theta360.guide/t/using-usb-api-mtp-with-libghoto2-and-python-bindings-on-macos-raspberry-pi-linux-ros/4521/48?u=craig).  
This is entirely his code.

## Documentation

Documentation for the module is available in English and Japanese at:

CHANGE URL TO OFFICIAL REPOSITORY

[HTML documentation available at temporary URL](
https://codetricity.github.io/multi_view_image_modified/)


## Modifications

I've added version and package information to Naoki's README based
on his [post](https://community.theta360.guide/t/using-usb-api-mtp-with-libghoto2-and-python-bindings-on-macos-raspberry-pi-linux-ros/4521/50?u=craig).

Change docstrings to conform to [PEP257](https://www.python.org/dev/peps/pep-0257/)


## Software Version

Naoki originally developed this on Ubuntu 16.04.

* gphoto2         2.5.23
* libgphoto2      2.5.25
* libgphoto2_port 0.12.0
* Python          2.7.12
* Python-gphoto2  2.2.2

As some of the software packages are old I added a Pipfile and pipenv.
libgphoto2 and gphoto2 were compiled from source.

To use pipenv.

```python
$ sudo apt install python-pip
$ python -m pip install pipenv
$ pipenv install gphoto2
$ pipenv shell
```



# LICENSE
This repository has a GPL-3.0 license.

## Files
main.py -> Connects to the desired THETA
pytheta.py -> The critical functions are written here

## Things to do
Here are the list of things to do. (will add later)
