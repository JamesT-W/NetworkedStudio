## HOW TO PASS YOUR DATA INTO OUR CUSTOM SERVER


1. Sort out your zone number in the team to differ our devices. For example, I have zone number 3.
Edit your ino code before flashing. In line 14, where we used to set our API keys for ThingSpeak:
```c
const String key = "3"; 
```
Change the numebr 3 to your zone number,eg. 1 or 2. (Currently it's only receiving 3 zones' data)

2. Go to your Particle console. Click New Integration > Webhook > Tick CustomJSON in the upper right corner

3. Paste this code into the field: 

```json
{
    "event": "CustomServer",
    "url": "http://47.88.159.162:8000/webapp/api/",
    "requestType": "POST",
    "json": {
      "key": "{{k}}",
      "temp": "{{1}}",
      "humid": "{{2}}",
      "light": "{{3}}",
      "time": "{{PARTICLE_PUBLISHED_AT}}"
    },
    "noDefaults": true
}
```

4. Click create webhook.

You can now flash the project firmware to your photon and it will post updates on the Customer server. Data are stored in the server as json files. Charts are still being worked on.THX
