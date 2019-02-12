# CF Sample App Python

A sample [Flask](http://flask.pocoo.org/) application to deploy to Cloud Foundry which works out of the box.

## Run locally

1. Install [Python](http://docs.python-guide.org/en/latest/starting/installation/)
1. Install Setuptools and pip (see guide above)
1. Install Virtualenv (acconplish this by running `pip install virtualenv`)
1. Run `virtualenv venv`
1. Run `source venv/bin/activate` on Mac OS X/Linux or`venv\Scripts\activate.bat` on windows
1. Run `pip install -r requirements.txt`
1. Run `python app.py`
1. Visit [http://localhost:3000](http://localhost:3000)

## Run in the cloud

1. Install the [cf CLI](https://github.com/cloudfoundry/cli#downloads)
1. Run `cf push my-python-app -m 128M --random-route`
1. Visit the given URL
