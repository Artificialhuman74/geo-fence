web: cd laptop-dashboard && SERVER_AUDIO=off gunicorn -w 1 --threads 8 -b 0.0.0.0:$PORT "server:create_wsgi_app()"
