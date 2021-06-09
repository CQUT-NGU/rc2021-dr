#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import glob

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
cwd = os.getcwd().replace('\\', '/')

# Gets the path to the execution file
pwd, filename = os.path.split(os.path.abspath(sys.argv[0]).replace('\\', '/'))
del filename

ioclist = glob.glob("{}/*.ioc".format(cwd))
try:
    f = open(ioclist[0], "r", encoding="utf-8")
    txtlist = f.read().split("\n")
    f.close()
except IndexError:
    print("Unfound *.ioc")
    exit()
del ioclist

elfname = ""  # The .elf filename
# Get project name
for txtline in txtlist:
    if "ProjectName" in txtline:
        elfname = txtline.split("=")[-1]
        break
del txtlist

config = oldconfig
# Gets the configuration file for the directory where the execution file resides
try:
    cfg_list = glob.glob("{}/*.cfg".format(cwd))
    config = cfg_list[0].replace('\\', '/')
    del cfg_list
except IndexError:
    try:
        cfg_list = glob.glob("{}/*.cfg".format(pwd))
        config = cfg_list[0].replace('\\', '/')
        del cfg_list
    except IndexError:
        print("Unfound *.cfg")
        exit()


def dealdir(dirname):
    """
    Add delimiter

    Args:
        dirname: The path to process
    Returns:
        The path after being processed
    """
    if "/" != dirname[-1] and "\\" != dirname[-1]:
        dirname += "/"
    return dirname


# Add delimiter
pwd = dealdir(pwd)
cwd = dealdir(cwd)
vsc = dealdir(vsc)
vscjson = dealdir(vscjson)


def launch(filename="launch.json"):
    """
    Set launch.json
    """

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    # Set .elf name
    txt = txt.replace(oldelfname, elfname)
    txt = txt.replace("./" + oldconfig, config)

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)

    return


def task(filename="tasks.json"):
    """
    Set tasks.json
    """

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)

    return


def c_cpp(filename="c_cpp_properties.json"):
    """
    Set c_cpp_properties.json
    """

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    # Set STM32 MCU macro
    try:
        asm_list = glob.glob("{}/startup*.s".format(cwd))
        dirname, s = os.path.split(asm_list[0])
        mcu_new = (s.split(".")[0]).split("_")[-1]
        mcu_new = mcu_new.upper().replace("XX", "xx")
        txt = txt.replace(mcu_old, mcu_new)
    except IndexError:
        print("Unfound startup*.s")
        exit()

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)

    return


def makefile(pwd, filename="Makefile"):
    """
    Set Makefile
    """

    openocd = "\topenocd -f " + pwd + "openocd.cfg -c init -c halt -c "
    cmd = (
        "flash:\n"
        + openocd
        + '"program $(BUILD_DIR)/$(TARGET).elf verify reset exit"\n'
    )
    cmd += "reset:\n" + openocd + "reset -c shutdown\n"

    if makefile_user not in os.listdir("."):
        with open(makefile_user, "w", encoding="utf-8") as f:
            f.write("C_SOURCES += \\\n$(wildcard *.c)\nC_INCLUDES += \\\n-I .\n")

    with open(filename, "r", encoding="utf-8") as f:
        txt = f.read()

    # Set inlcude user' Makefile
    txt_inc = "-include {}\n".format(makefile_user)
    tmp = "# default"
    if txt_inc not in txt:
        txt = txt.replace(tmp, "{}\n{}".format(txt_inc, tmp))

    # Deal with the end
    end = "EOF"
    txtlist = txt.split(end)
    txt = txtlist[0] + end
    if " ***" in txtlist[-1]:
        txt += " ***"
    txt += "\n" + cmd

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
