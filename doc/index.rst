.. _camera-marccd:

MarCCD
------

.. image:: marccd.png

Intoduction
```````````

The SX165 features a round, 165 mm diameter active area, and a versatile, high resolution CCD chip. It is the ideal X-ray detector for research applications with both synchrotrons and rotating anode X-ray sources.


Prerequisite
````````````
The MarCCD software server should be started on the MarCCD host computer, by running the command:

.. code-block:: bash

	marccd -r

Then you can launch your lima/marccd client on another host, as the MarCCD server can be reached by network


Initialisation and Capabilities
````````````````````````````````
In order to help people to understand how the camera plugin has been implemented in LImA this section
provide some important information about the developer's choices.

Camera initialisation
......................

There are 4 parameter to be filled by your Lima client:

	- The IPAddress of the host where the marccd server is running
	- The port of the marccd server process
	- The detector target path: the path where will be saved the marccd image files
	- Reader timeout: in ms, the timeout after which the plugin will be in fault if no marccd image file is present 

Std capabilites
................

This plugin has been implemented in respect of the mandatory capabilites but with some limitations according 
to some programmer's  choices.  We only provide here extra information for a better understanding
of the capabilities for the MarCCD camera.

* HwDetInfo
  
 - Max image size is : 4096 * 4096
 - 16 bit unsigned type is supported

* HwSync

  trigger type supported are:
	- IntTrig
  
Optional capabilites
........................

* HwBin
	- 2 * 2
	- 4 * 4
	- 8 * 8

* HwRoi

Configuration
`````````````

No Specific hardware configuration are needed

How to use
````````````
here is the list of accessible fonctions to configure and use the MarCCD  detector:

.. code-block:: cpp

	void getDetectorImageSize(Size& size);
	void setImagePath(const std::string& path);
	const std::string& getImagePath(void);
	void setImageFileName(const std::string& imgName);
	const std::string& getImageFileName();
	void setImageIndex(int newImgIdx);
	int getImageIndex();
	int getFirstImage();

	bool isStopSequenceFinished();
	void saveBGFrame(bool);

	void setBeamX(float);
	float getBeamX();
	void setBeamY(float);
	float getBeamY();
	void setDistance(float);
	float getDistance();

	void setWavelength(float);
	float getWavelength();




