import subprocess
import os
import re
from typing import Iterable, Dict, Tuple
from project_xml_generation import generate_project_xml


REQUIRED_MODULES = {"taproot:build", "taproot:modm-project.xml", "taproot:ci-scripts"}
ENFORCED_OPTIONS = {"rebuild_modm": True}

p = b"usage: lbuild [-h] [-r REPO] [-c CONFIG] [-p PATH] [-D OPTION]\n              [--collect COLLECTOR] [-v] [--plain] [--version]\n              {discover,discover-options,search,validate,build,clean,init,update,dependencies}\n              ...\n\nBuild source code libraries from modules.\n\noptional arguments:\n  -h, --help            show this help message and exit\n  -r REPO, --repository REPO\n                        Repository file(s) which should be available for the\n                        current library. The loading of repository files from\n                        a VCS is only supported through the library\n                        configuration file.\n  -c CONFIG, --config CONFIG\n                        Project configuration file. Specifies the required\n                        repositories, modules and options (default:\n                        'project.xml').\n  -p PATH, --path PATH  Path in which the library will be generated (default:\n                        '.').\n  -D OPTION, --option OPTION\n                        Additional options. Options given here will be merged\n                        with options from the configuration file and will\n                        overwrite the configuration file definitions.\n  --collect COLLECTOR   Additional collectors. Values given here will be\n                        merged with collectors from the configuration file.\n  -v, --verbose\n  --plain               Disable styled output, only output plain ASCII.\n  --version             Print the lbuild version number and exit.\n\nActions:\n  {discover,discover-options,search,validate,build,clean,init,update,dependencies}\n    discover            Render the available repository tree with modules and\n                        options. You may need to provide options to see the\n                        entire tree!\n    discover-options    Display all known option names, current values,\n                        allowed inputs and short descriptions.\n    search              Search the descriptions of the repository tree and\n                        render the results as a partial tree and the matching\n                        lines of the descriptions.\n    validate            Validate the library configuration and data inputs\n                        with the given options.\n    build               Generate the library source code blob with the given\n                        options.\n    clean               Remove previously generated files.\n    init                Load remote repositories into the cache folder.\n    update              Update the content of remote repositories in the cache\n                        folder.\n    dependencies        Generate a grahpviz representation of the module\n                        dependencies.\nNo command specified!\n"

print(p.dencode("utf-8"))

def collect_generation_information() -> Dict[str, Dict[str, Iterable[str]]]:
    generation_information = {}
    lbuild_discover = lambda args=[]: subprocess.run(['lbuild', *args, 'discover'], env={**dict(os.environ), **{"PYTHONIOENCODING": "utf-8"}}, shell=True, capture_output=True)

    device_discover = lbuild_discover()
    print(device_discover.stdout)
    print(device_discover.stderr)
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

        options = set(re.findall('Option\((.*)\) = (?:true|false|yes|no)', device_discover))
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
