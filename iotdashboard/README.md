# Setup Instructions:

1. Run following commands in a terminal to install python dependencies (open the terminal with current location outside `Firebird-V` directory, otherwise it will get added to GitHub):

        $ pip3 install virtualenv
        $ virtualenv venv
        $ source venv/bin/activate
        $ pip install -r Firebird-V/iotdashboard/requirements.txt

2. Follow instructions till Step 2 (including it) from https://www.digitalocean.com/community/tutorials/how-to-install-and-secure-redis-on-ubuntu-20-04 to install redis server

3. To start server, run following command in a terminal (assuming the terminal is opened at the same directory level of `Firebird-V`):

        $ virtualenv venv
        $ source venv/bin/activate
        $ cd Firebird-V/iotdashboard/
        $ python manage.py runserver

4. Now open your favourite browser and enter the URL `127.0.0.1:8000`
