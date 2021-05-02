#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys

# Target configuration directory
vscjson = "vscjson"
# Source configuration directory
vsc = ".vscode"
# Default .elf filename
oldelfname = "ELFNAME"
# Default .cfg filename
oldconfig = "openocd.cfg"
# Default STM32 MCU
mcu_old = "STM32MCU"
# Default uaser' Makefile
makefile_user = "Makefile_user.txt"

# Gets the current path
cwd = os.getcwd()
# Remove the last delimiter
if cwd[-1] == "\\" or cwd[-1] == "/":
    cwd = cwd[:-1:]

# Get the execution file name
binpwd = sys.argv[0].split("\\")[-1].split("/")[-1]
# Gets the path to the execution file
pwd = sys.argv[0].replace(binpwd, "").replace("\\", "/")
del binpwd

iocname = ""  # The .ioc filename
# Gets the .ioc file for the current path
ioclist = os.listdir(cwd)
for iocline in ioclist:
    if ".ioc" in iocline:
        iocname = iocline
        break
del ioclist
with open(iocname, "r", encoding="utf-8") as f:
    txtlist = f.read().split("\n")
del iocname

elfname = ""  # The .elf filename
# Get project name
for txtline in txtlist:
    if "ProjectName" in txtline:
        elfname = txtline.split("=")[-1]
        break
del txtlist

config = oldconfig
# Gets the configuration file for the directory where the execution file resides
cfglist = os.listdir(pwd)
for cfgline in cfglist:
    if ".cfg" in cfgline:
        config = cfgline
        break
del cfglist


def dealpwd(pwd):
    """
    Add delimiter

    Args:
        pwd: The path to process
    Returns:
        The path after being processed
    """
    if "/" != pwd[-1] and "\\" != pwd[-1]:
        pwd += "/"
    return pwd


# Add delimiter
pwd = dealpwd(pwd)
cwd = dealpwd(cwd)
vsc = dealpwd(vsc)
vscjson = dealpwd(vscjson)


def launch():
    """
    Set launch.json
    """
    filename = "launch.json"

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    # Set .elf name
    txt = txt.replace(oldelfname, elfname)
    txt = txt.replace("./" + oldconfig, pwd + config)

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)

    return


def task():
    """
    Set tasks.json
    """
    filename = "tasks.json"

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)

    return


def c_cpp():
    """
    Set c_cpp_properties.json
    """
    filename = "c_cpp_properties.json"

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    # Set STM32 MCU macro
    file_list = os.listdir(cwd)
    for s in file_list:
        if ".s" in s and "startup" in s:
            mcu_new = (s.split(".")[0]).split("_")[-1]
            mcu_new = mcu_new.upper().replace("XX", "xx")
            txt = txt.replace(mcu_old, mcu_new)
            break
    del file_list

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)

    return


def makefile(pwd):
    """
    Set Makefile
    """
    filename = "Makefile"

    openocd = "\topenocd -f " + pwd + "openocd.cfg -c init -c halt -c "
    cmd = (
        "flash:\n"
        + openocd
        + '"program $(BUILD_DIR)/$(TARGET).hex verify reset exit"\n'
    )
    cmd += "reset:\n" + openocd + "reset -c shutdown\n"
    del openocd

    if makefile_user not in os.listdir("."):
        with open(makefile_user, "w", encoding="utf-8") as f:
            f.write("C_SOURCES += \\\n$(wildcard *.c)\nC_INCLUDES += \\\n-I .\n")

    with open(filename, "r", encoding="utf-8") as f:
        txt = f.read()

    # Set inlcude user' Makefile
    txt_inc = "-include {}\n".format(makefile_user)
    tmp = "# compile"
    if txt_inc not in txt:
        txt = txt.replace(tmp, "{}\n{}".format(txt_inc, tmp))
    del tmp
    del txt_inc

    # Deal with the end
    end = "EOF"
    txtlist = txt.split(end)
    del txt

    txt = txtlist[0] + end
    del end

    if " ***" in txtlist[-1]:
        txt += " ***"
    del txtlist

    txt += "\n" + cmd
    del cmd

    with open(filename, "w", encoding="utf-8") as f:
        f.write(txt)

    return


def vscinit():
    """
    init .vscode
    """
    try:
        os.mkdir(vsc)
    except FileExistsError:
        pass
    except Exception as e:
        print(e)
        exit()

    return


if __name__ == "__main__":
    vscinit()
    c_cpp()
    launch()
    task()
    makefile(pwd)
    # Show log
    print("config:", pwd + "openocd.cfg")
