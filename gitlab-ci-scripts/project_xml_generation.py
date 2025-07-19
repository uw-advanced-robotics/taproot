import os
import sys
import itertools
import hashlib
from typing import Iterable, Dict, Any

PROJECT_XML_TEMPLATE_PATH = "ci-project-template.xml"

def hash_dict(dict: Dict[str, Any]):
    hasher = hashlib.sha256()

    for opt, val in sorted(dict.items(), key=lambda e:e[0]):
        hasher.update(f"{opt}={val}".encode())

    return hasher.hexdigest()

def hash_list(l: Iterable[str]):
    hasher = hashlib.sha256()

    for e in sorted(l):
        hasher.update(e.encode())

    return hasher.hexdigest()

def generate_project_xml(board: str, options: Dict[str, Any], modules: Iterable[str]) -> str:
    output = f"{board}-{hash_dict(options)}-{hash_list(modules)}-project.xml"
    with open(PROJECT_XML_TEMPLATE_PATH) as f:
        file = f.read()
        file = file.replace("$BOARD$", board)
        file = file.replace("$OPTIONS$", 
            "\n\t".join(
                f"<option name=\"{o}\">{v}</option>" for o, v in options.items()
            )
        )
        file = file.replace("$MODULES$", 
            "\n\t".join(
                f"<module>{m}</module>" for m in modules
            )
        )

        with open(output, "w") as out:
            out.write(file)
    
    return output


if __name__ == "__main__":
    args = list(sys.argv)
    _, device, modules, *_ = args

    modules = modules.split(",")

    options = {}
    if len(args) == 4:
        options = {
            opt.split("=")[0]: opt.split("=")[1] == "True" for opt in args[3].split(",")
        }
    
    xml = generate_project_xml(device, options, modules)
    print(xml)