## Build

Be inside openr3flector_code (here)

> cd openr3flector_code

Then do 

> make build

If you got a st-link, attach it and run

> make flash

There is also a OpenOCD version, tweak it if needed.


## QoL things

When you're building and flashing it, just do

> make rebuild

to clean and build.

### Lifehax progging
make a symlink from code root dir to build/compile_commands.json by 

> ln -s build/compile_commands.json compile_commands.json

Which will allow clangd to know which references are where.