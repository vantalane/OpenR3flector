## Developer Resources

* [INDI API](http://www.indilib.org/api/index.html)
* [INDI Developer Manual](http://indilib.org/develop/developer-manual.html)
* [Tutorials](http://indilib.org/develop/tutorials.html)
* [Developers Forum](http://indilib.org/forum/development.html)
* [Developers Chat](https://riot.im/app/#/room/#kstars:matrix.org)


# INDI driver code for r3flector opencontrol

This driver code allows you to control the r3flector dish system as if it is a telescope. 
Some planetarium applications such as KStars control motorized alt/az telescopes through INDI, an opensource protocol framework. 

For INDI compatible apps to control your system, you need the compiled binary, and a declaration of this device that tells the programs 
where the binary is found, and what the device is named.

The binary is named "indi_r3_telescope". 

> sudo ln -s /home/....../....../r3_indi_driver_sk.xml /usr/share/indi/

indi xml stuff is in 
usr/share/indi/

need to make the driver accesible in the xml <br>
sudo ln -s /home/../../.../satenne_r3_opencontrol/code/indi_opencontrol_driver/indi_r3_telescope /usr/local/bin/

# To build

If you don't have a build dir yet:

> mkdir build

go into build dir
> cd build

then:
> cmake ..

then build:
>make

Then place the binary wherever the INDI's XML file points at.
> cp indi_r3_telescope ../

## Important for how INDI works

You system's indi server has a XML list of devices and their driver binaries.
When you add this device in that list, you need give the path to the binary as well. 