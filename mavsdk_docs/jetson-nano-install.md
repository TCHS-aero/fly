[Original Source](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/jetson-nano-install.html)

# Jetson Nano Install[¶](#jetson-nano-install "Link to this heading")

## Ubuntu 18.04[¶](#ubuntu-18-04 "Link to this heading")

To install MAVSDK-Python on a Jetson Nano with Ubuntu 18.04 (which is old and pas end-of-life, by the way), you need to get a newer version of Python 3 and make sure pip is up-to-date.

Install Python 3.8:

```
sudo apt update
sudo apt install python3.8
```

Upgrade pip:

```
python3.8 -m pip install --upgrade pip
```

Now install mavsdk:

```
python3.8 -m pip install --upgrade mavsdk
```

## Ubuntu 20.04[¶](#ubuntu-20-04 "Link to this heading")

The normal instructions should work with Ubuntu 20.04:

Install mavsdk:

```
python3 -m pip install --upgrade mavsdk
```