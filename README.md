# NetworkedStudio
SCC330 Networked Studio
Contributors: Viktor Kolev, Eduard Schlotter, James Thomas-Woodhouse, Cyrus Gao

https://docs.particle.io/reference/firmware/photon/#particle-publish- READ THE DOCUMENTATION GENTELMEN



If your publish events aren't updating properly, this is why:

NOTE: Currently, a device can publish at rate of about 1 event/sec, with bursts of up to 4 allowed in 1 second. Back to back burst of 4 messages will take 4 seconds to recover.


/********ParticleDev folder structure reference********/

# TiltPublisherProject

A Particle project named TiltPublisherProject

## Welcome to your project!

Every new Particle project is composed of 3 important elements that you'll see have been created in your project directory for TiltPublisherProject.

#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.ino``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE.

## Adding additional files to your project

#### Projects with multiple sources
If you would like add additional files to your application, they should be added to the `/src` folder. All files in the `/src` folder will be sent to the Particle Cloud to produce a compiled binary.

#### Projects with external libraries
If your project includes a library that has not been registered in the Particle libraries system, you should create a new folder named `/lib/<libraryname>/src` under `/<project dir>` and add the `.h` and `.cpp` files for your library there. All contents of the `/lib` folder and subfolders will also be sent to the Cloud for compilation.

## Compiling your project

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`

/***************************************************************************************/


<<<<<<<<<HOW TO INTEGRATE THE DATAHISTORY-THINGSPEAK BRANCH INTO YOUR OWN PROJECT>>>>>>>>

Simply compiling and flashing the code to your device is not enough. Follow these steps to get your photon to publish to this project's ThingSpeak Channel (https://thingspeak.com/channels/350722)

1. Go to https://console.particle.io/integrations

2. Click New Integration > Webhook > Tick CustomJSON in the upper right corner

3. Paste this code into the field: 

{
    "event": "thingSpeakWrite_",
    "url": "https://api.thingspeak.com/update",
    "requestType": "POST",
    "form": {
        "api_key": "{{k}}",
        "field1": "{{1}}",
        "field2": "{{2}}",
        "field3": "{{3}}",
        "field4": "{{4}}",
        "field5": "{{5}}",
        "field6": "{{6}}",
        "field7": "{{7}}",
        "field8": "{{8}}",
        "lat": "{{a}}",
        "long": "{{o}}",
        "elevation": "{{e}}",
        "status": "{{s}}"
    },
    "mydevices": true,
    "noDefaults": true
}

4. Click create webhook.

You can now flash the project firmware to your photon and it will post updates on the ThingSpeak channel.