import os
import re
import shutil
import subprocess
import sys

FILE = "CMakeLists.txt"
FILE2 = "examples/CMakeLists.txt"
comment_files = [
    "helper/cooperativePerception-helper.cc",
    "model/Facilities/vdpopencda.cc",
    "model/Applications/cooperativePerception.cc",
    "model/utilities/opencda-sensor.cc",
    "helper/cooperativePerception-helper.h",
    "model/Applications/cooperativePerception.h",
    "model/Facilities/vdpopencda.h",
    "model/utilities/opencda-sensor.h",
]

uncomment_files = [
    "helper/v2xEmulator-helper.cc",
    "helper/simpleCAMSender-helper.cc",
    "model/Applications/simpleCAMSender-gps-tc.cc",
    "model/Applications/v2xEmulator.cc",
    "helper/v2xEmulator-helper.h",
    "helper/simpleCAMSender-helper.h",
    "model/Applications/simpleCAMSender-gps-tc.h",
    "model/BTP/simple-btp-application.h",
]

header_content = """find_package(protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
link_libraries(${Protobuf_LIBRARIES})

find_package(gRPC REQUIRED)
include_directories(${gRPC_INCLUDE_DIRS})
link_libraries(${gRPC_LIBRARIES})
link_libraries (protobuf::libprotobuf
    gRPC::grpc
    gRPC::grpc++)"""

def comment_out(file, pattern):
    with open(file, 'r') as f:
        lines = f.readlines()
    with open(file, 'w') as f:
        for line in lines:
            if re.match(rf'^[^#]*{pattern}.*', line):
                f.write('# ' + line)
            else:
                f.write(line)

def uncomment(file, pattern):
    with open(file, 'r') as f:
        lines = f.readlines()
    with open(file, 'w') as f:
        for line in lines:
            f.write(re.sub(rf'^# (.*{pattern}.*)', r'\1', line))


def comment_sections(file_path, section_names):
    # Read the content of the CMakeLists.txt file
    with open(file_path, 'r') as file:
        content = file.read()

    # Iterate over each section name to comment out
    for section in section_names:
        # Regular expression to find the block to comment out
        pattern = re.compile(rf'(build_lib_example\(\s*NAME\s*{re.escape(section)}\s*.*?LIBRARIES_TO_LINK\s*.*?\))', re.DOTALL)

        # Add comments to the matched block
        content = pattern.sub(lambda match: "\n".join([f"# {line}" for line in match.group(0).splitlines()]), content)

    # Write the modified content back to the file
    with open(file_path, 'w') as file:
        file.write(content)

def comment_section_examples(file_path):
    inside_block = False
    block_lines = []
    libcarla_found = False

    with open(file_path, 'r') as file:
        lines = file.readlines()

    new_lines = []

    for line in lines:
        # Detect the start of the build_lib_example block
        if 'build_lib_example(' in line:
            inside_block = True
            block_lines = [line]  # Start capturing the block
            libcarla_found = False  # Reset libcarla_found flag

        elif inside_block:
            block_lines.append(line)

            # Detect if ${libcarla} is in the block
            if '${libcarla}' in line:
                libcarla_found = True

            # Detect the end of the build_lib_example block
            if ')' in line:
                inside_block = False

                # If ${libcarla} was found, comment out the whole block
                if libcarla_found:
                    block_lines = [f"# {l}" if not l.startswith("#") else l for l in block_lines]

                # Add the block to the final lines
                new_lines.extend(block_lines)

        else:
            new_lines.append(line)

    # Write the modified content back to the file
    with open(file_path, 'w') as file:
        file.writelines(new_lines)

def uncomment_sections(file_path, section_names):
    # Read the content of the CMakeLists.txt file
    with open(file_path, 'r') as file:
        content = file.read()

    # Iterate over each section name to uncomment
    for section in section_names:
        # Regular expression to find the block to uncomment
        pattern = re.compile(rf'(build_lib_example\(\s*NAME\s*{re.escape(section)}\s*.*?LIBRARIES_TO_LINK\s*.*?\))', re.DOTALL)

        # Remove comments from the matched block
        content = pattern.sub(lambda match: "\n".join([line.lstrip('# ') if line.lstrip().startswith('#') else line for line in match.group(0).splitlines()]), content)

    # Write the modified content back to the file
    with open(file_path, 'w') as file:
        file.write(content)

def uncomment_section_examples(file_path, exclude_sections=None):
    # exclude_sections: list of example NAMEs that must be left commented out
    # (e.g. blocks intentionally disabled by comment_sections). Without this,
    # every block containing ${libcarla} gets uncommented, which silently
    # reverses comment_sections() when those blocks also reference ${libcarla}
    exclude_sections = exclude_sections or []

    inside_block = False
    block_lines = []
    libcarla_found = False
    excluded = False

    with open(file_path, 'r') as file:
        lines = file.readlines()

    new_lines = []

    for line in lines:
        # Detect the start of the build_lib_example block
        if 'build_lib_example(' in line:
            inside_block = True
            block_lines = [line]  # Start capturing the block
            libcarla_found = False  # Reset libcarla_found flag
            excluded = False  # Reset exclusion flag

        elif inside_block:
            block_lines.append(line)

            # Detect if ${libcarla} is in the block
            if '${libcarla}' in line:
                libcarla_found = True

            # Detect if this block's NAME is in the exclusion list.
            # Strip any leading comment marker so it matches whether the
            # block is currently commented or not
            stripped = line.lstrip('# ').rstrip()
            for section in exclude_sections:
                if re.match(rf'NAME\s+{re.escape(section)}$', stripped):
                    excluded = True

            # Detect the end of the build_lib_example block
            if ')' in line:
                inside_block = False

                # If ${libcarla} was found and the block is not excluded, uncomment the whole block
                if libcarla_found and not excluded:
                    block_lines = [l.lstrip('# ') for l in block_lines]

                # Add the block (either uncommented or left as is) to the final lines
                new_lines.extend(block_lines)

        else:
            # If we are not inside a block, add the line as-is
            new_lines.append(line)

    # Write the modified content back to the file
    with open(file_path, 'w') as file:
        file.writelines(new_lines)


def prepend_header_if_absent(file, header_content):
    with open(file, 'r') as f:
        content = f.read()
    if "find_package(protobuf REQUIRED)" not in content:
        with open(file, 'w') as f:
            f.write(header_content + "\n" + content)


def delete_header_if_present(file, header_content):
    with open(file, 'r') as f:
        content = f.read()
    if header_content in content:
        with open(file, 'w') as f:
            f.write(content.replace(header_content, ""))


def append_after_pattern(file, pattern, content):
    with open(file, 'r') as f:
        lines = f.readlines()
    with open(file, 'w') as f:
        for line in lines:
            if line == "    " + content + "\n":
                continue
            f.write(line)
            if re.search(pattern, line):
                f.write("    " + content + "\n")



def remove_after_pattern(file, pattern):
    with open(file, 'r') as f:
        lines = f.readlines()
    # remove the line with the pattern
    with open(file, 'w') as f:
        for line in lines:
            if line == "    " + pattern + "\n":
                continue
            f.write(line)


def copytree_compat(src, dst):
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def switch_to_carla():
    ns_3_dir = os.getcwd()

    subprocess.run(["mkdir", "src-backup"])
    subprocess.run(["cp", "-r", "src/", "src-backup/"])

    # Switch CMakeLists.txt and PRRsup
    os.chdir('src/automotive/')
    copytree_compat('model/Measurements/', 'aux-files/Measurements-base/')
    copytree_compat('aux-files/Measurements-CARLA/', 'model/Measurements/')


    for pattern in uncomment_files:
        comment_out(FILE, pattern)

    for pattern in comment_files:
        uncomment(FILE, pattern)
    
    sections_to_comment = [
        'v2x-emulator',
        'v2v-80211p-gps-tc-example',
        'v2v-80211p-gps-tc-dcc'
    ]

    # Uncomment ${libcarla} blocks first, skipping the ones we intend to
    # disable, then comment out the disabled sections last. Order matters:
    # uncomment_section_examples() would otherwise re-enable any disabled
    # block that references ${libcarla}.
    uncomment_section_examples(FILE2, exclude_sections=sections_to_comment)

    comment_sections(FILE2, sections_to_comment)

    # prepend_header_if_absent(FILE, header_content)

    append_after_pattern(FILE, r'\${libtraci}', r'${libcarla}')

    if os.path.exists("aux-files/current-mode.txt"):
        os.remove("aux-files/current-mode.txt")

    with open("aux-files/current-mode.txt", 'w') as f:
        f.write("CARLA\n")

    os.chdir(ns_3_dir)
    os.chdir("src/carla/proto/")
    subprocess.run(["./buildProto.sh"])
    os.chdir(ns_3_dir)

    print("Successfully switched to ms-van3t-CARLA!")

def switch_to_base():
    ns_3_dir = os.getcwd()

    """
    # Switch CMakeLists.txt and PRRsup
    os.chdir('src/automotive/')
    copytree_compat('model/Measurements/', 'aux-files/Measurements-CARLA/')
    copytree_compat('aux-files/Measurements-base/', 'model/Measurements/')

    for pattern in comment_files:
        comment_out(FILE, pattern)

    for pattern in uncomment_files:
        uncomment(FILE, pattern)

    
    sections_to_comment = [
        'v2x-emulator',
        'v2v-80211p-gps-tc-example',
        'v2v-80211p-gps-tc-dcc',
        'v2v-simple-cam-exchange-80211p'
    ]
    

    uncomment_sections(FILE2, sections_to_comment)

    comment_section_examples(FILE2)

    # delete_header_if_present(FILE, header_content)

    remove_after_pattern(FILE, '${libcarla}')
    """
    
    sections_to_comment = [
        'v2x-emulator',
        'v2v-80211p-gps-tc-example',
        'v2v-80211p-gps-tc-dcc'
    ]
    
    os.chdir('src/automotive/')

    uncomment_sections(FILE2, sections_to_comment)

    comment_section_examples(FILE2)

    # delete_header_if_present(FILE, header_content)

    remove_after_pattern(FILE, '${libcarla}')

    os.chdir(ns_3_dir)
    subprocess.run(["rm", "-rf", "src/"])
    subprocess.run(["mv", "src-backup/src", "src"])
    subprocess.run(["rm", "-rf", "src-backup/"])
    
    os.chdir('src/automotive/')

    if os.path.exists("aux-files/current-mode.txt"):
        os.remove("aux-files/current-mode.txt")

    with open("aux-files/current-mode.txt", 'w') as f:
        f.write("base\n")

    os.chdir(ns_3_dir)
    os.chdir("src/carla/proto/")
    subprocess.run(["./buildProto.sh"])
    os.chdir(ns_3_dir)

    print("Successfully switched to base ms-van3t!")



def main():

    # if the first argument is 'CARLA', switch to CARLA mode
    if len(sys.argv) > 1 and sys.argv[1] == 'CARLA':
        switch_to_carla()
    elif len(sys.argv) > 1 and sys.argv[1] == 'BASE':
        switch_to_base()



if __name__ == "__main__":
    main()