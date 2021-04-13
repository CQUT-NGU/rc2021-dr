#!/usr/bin/env python3
import os
import sys

vscjson = "vscjson"
vsc = ".vscode"

cwd = os.getcwd()
if cwd[-1] == "\\" or cwd[-1] == "/":
    cwd = cwd[:-1:]

binpwd = sys.argv[0].split("\\")[-1].split("/")[-1]
pwd = sys.argv[0].replace(binpwd, "").replace("\\", "/")
del binpwd

oldelfname = "ELFNAME"
iocname = ""
ioclist = os.listdir(cwd)
for iocline in ioclist:
    if ".ioc" in iocline:
        iocname = iocline
        break
del ioclist
elfname = ""
with open(iocname, "r", encoding="utf-8") as f:
    txtlist = f.read().split("\n")
del iocname
for txtline in txtlist:
    if "ProjectName" in txtline:
        elfname = txtline.split("=")[-1]
        break
del txtlist

oldconfig = "openocd.cfg"
config = oldconfig
cfglist = os.listdir(pwd)
for cfgline in cfglist:
    if ".cfg" in cfgline:
        config = cfgline
        break
del cfglist


def dealpwd(pwd):
    if "/" != pwd[-1] and "\\" != pwd[-1]:
        pwd += "/"
    return pwd


pwd = dealpwd(pwd)
cwd = dealpwd(cwd)
vsc = dealpwd(vsc)
vscjson = dealpwd(vscjson)


def launch():
    filename = "launch.json"

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    txt = txt.replace(oldelfname, elfname)
    txt = txt.replace("./" + oldconfig, pwd + config)

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)
    del txt
    del filename

    return


def task():
    filename = "tasks.json"

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

    with open(cwd + vsc + filename, "w", encoding="utf-8") as f:
        f.write(txt)
    del txt
    del filename

    return


def c_cpp():
    filename = "c_cpp_properties.json"
    mcu_old = "STM32MCU"

    with open(pwd + vscjson + filename, "r", encoding="utf-8") as f:
        txt = f.read()

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
    del txt
    del filename

    return


def makefile(pwd):
    filename = "Makefile"
    makefile_user = "Makefile_user.txt"

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
            f.write("C_SOURCES += \\\n$(wildcard *.c)\nC_INCLUDES += \\\n-I.\n")

    with open(filename, "r", encoding="utf-8") as f:
        txt = f.read()

    txt_inc = "-include {}\n".format(makefile_user)
    tmp = "# compile"
    if txt_inc not in txt:
        txt = txt.replace(tmp, "{}\n{}".format(txt_inc, tmp))
    del tmp
    del txt_inc

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
    del filename
    del txt

    return


def vscinit():
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
    print("config:", pwd + "openocd.cfg")
