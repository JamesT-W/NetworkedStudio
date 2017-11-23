# NetworkedStudio
SCC330 Networked Studio
Contributors: Viktor Kolev, Eduard Schlotter, James Thomas-Woodhouse, Cyrus Gao

https://docs.particle.io/reference/firmware/photon/#particle-publish- READ THE DOCUMENTATION GENTLEMEN



If your publish events aren't updating properly, this is why:

NOTE: Currently, a device can publish at rate of about 1 event/sec, with bursts of up to 4 allowed in 1 second. Back to back burst of 4 messages will take 4 seconds to recover.


#CupTilting

Posts cup %fill to VMServer.
