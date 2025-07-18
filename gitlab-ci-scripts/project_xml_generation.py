import os
import sys
import itertools
from typing import Iterable, Dict

PROJECT_XML_TEMPLATE_PATH = "ci-project-template.xml"

def generate_project_xml(board: str, options: Dict[str, bool], modules: Iterable[str]) -> str:
    output = f"{board}-{hash(frozenset(options.items()))}-{hash(frozenset(modules))}-project.xml"
    with open(PROJECT_XML_TEMPLATE_PATH) as f:
        file = f.read()
        file = file.replace("$BOARD$", board)
        file = file.replace("$OPTIONS$", 
            "\n".join(
                f"<option name=\"{o}\">{v}<\\option>" for o, v in options.items()
            )
        )
        file = file.replace("$MODULES$", 
            "\n".join(
                f"<module>{m}<\\module>" for m in modules
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