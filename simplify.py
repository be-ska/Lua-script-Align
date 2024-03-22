# python script to remove unnecessary characters from a Lua script (comments, spaces, new lines)

import sys
import re

# Check if the input file name is provided
if len(sys.argv) < 2:
    print("Please provide the input file name as an argument.")
    sys.exit(1)

input_file_name = sys.argv[1]
first = True

# Open the input and output files
with open(input_file_name, 'r') as infile, open(f"formatted-{input_file_name}", 'w') as outfile:
    for line in infile:
        if first:
            first = False
            outfile.write(line)
        else:
            # Remove everything after "--"
            line = re.sub(r'--.*$', '', line)
            
            # Remove empty lines, including those with spaces
            if line.strip():
                # Replace two or more consecutive spaces with a single space
                # First, replace all spaces with a single space
                line = ' '.join(line.split())
                # Then, ensure the newline character is preserved
                outfile.write(line + '\n')
