import subprocess
import os
import re
from typing import Iterable, Dict, Tuple
from project_xml_generation import generate_project_xml


REQUIRED_MODULES = {"taproot:build", "taproot:modm-project.xml", "taproot:ci-scripts"}
ENFORCED_OPTIONS = {"rebuild_modm": True}

def collect_generation_information() -> Dict[str, Dict[str, Iterable[str]]]:
    generation_information = {}
    lbuild_discover = lambda args=[]: subprocess.run(['lbuild', *args, 'discover'], env={**dict(os.environ), **{"PYTHONIOENCODING": "utf-8"}}, capture_output=True, cwd=os.getcwd())

    device_discover = lbuild_discover()
    devices = re.findall(b'Option\(dev_board\) = .* in \\[(.*)\\]', device_discover.stdout)[0].decode("utf-8").split(", ")
    print(f"Found {len(devices)} devices: {devices}")

    jobs = 0
    for device in devices:
        xml = generate_project_xml(device, {}, [])

        device_discover = lbuild_discover(["-c", xml]).stdout.decode("utf-8")
        lines = device_discover.split("\n")

        modules = set(re.findall('Module\((.*)\)', device_discover))
        module_lines = {
            module: [i for i, line in enumerate(lines) if f"Module({module})" in line][0] for module in modules
        }
        modules_to_select = modules - REQUIRED_MODULES

        options = set(re.findall('Option\((.*)\) = (?:[tT]rue|[fF]alse|[yY]es|[nN]o)', device_discover))
        options_to_toggle = options - ENFORCED_OPTIONS.keys()
        option_lines = {
            option: [i for i, line in enumerate(lines) if f"Option({option})" in line][0] for option in options_to_toggle
        }

        setups = {
            "all": options_to_toggle
        }

        for module in modules_to_select:
            setups[module] = set()

        for option in options_to_toggle:
            module = None

            for m, l in sorted(module_lines.items(), key=lambda e: e[0]):
                if module is None or l < option_lines[option]:
                    module = m
            setups[module].add(option)

        print(f"{device} has {len(options_to_toggle)} options and {len(modules_to_select) + 1} modules for {sum(2 ** len(v) for v in setups.values())} total jobs.")
        generation_information[device] = setups
        os.remove(xml)
        jobs += sum(2 ** len(v) for v in setups.values())

    print(f"Total jobs to run: {jobs}")

    return generation_information

if __name__ == "__main__":
    print("Collecting generation information...")

    info = collect_generation_information()

    print()
    for device, config in sorted(info.items(), key=lambda e:e[0]):
        print(f"{device}:")
        for module, options in sorted(config.items(), key=lambda e: e[0]):
            print(f"\t{module}: [{', '.join(options)}]")
