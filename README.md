# NetworkedStudio
SCC330 Networked Studio
Contributors: Viktor Kolev, Eduard Schlotter, James Thomas-Woodhouse, Cyrus Gao

https://docs.particle.io/reference/firmware/photon/#particle-publish- READ THE DOCUMENTATION GENTLEMEN



If your publish events aren't updating properly, this is why:

NOTE: Currently, a device can publish at rate of about 1 event/sec, with bursts of up to 4 allowed in 1 second. Back to back burst of 4 messages will take 4 seconds to recover.

#Compiling this project
Before you compile, make sure the key String(line 17) corresponds to which lab zone your device will be in.

#Server notes

https://github.com/cylepsy/SmartLabServer

Check this repo out. The latest implementation is the untitiled folder. You would only need that to run a server.

How to run the server:

I 've installed all the environment needs to run the servere (Python2.7 and Django). If the server is not divided by the accounts we logged in, it should work.
Heres the method to run the server:
when navigated to the directory where manage.py is, run this:
```
python manage.py runserver 0.0.0.0:8000
```

run on 0.0.0.0 should run the server file to all ips that the vm server binding to if there's no firewall blocking it.
for more detail please check the Django Documents:

https://www.djangoproject.com/start/

