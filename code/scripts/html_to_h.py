import os
import sys

def main(html_path: str, h_path: str):
    COLUMNS = 16

    with open(html_path) as html_file:
        html = html_file.read()
    
    # Convert each character in the HTML file to a hex code in the form 0xff
    html_hex = []
    for i, char_ord in enumerate(html.encode()):
        if i % 16 == 0:
            html_hex.append([])
        if char_ord < 0 or char_ord > 127:
            print(f"Warning: non-ascii character: {char_ord} (index={i})")

        html_hex[-1].append("0x{:02x}".format(char_ord))
    
    # Format the hex codes in C array syntax 
    html_hex_c_array = f"{{\n    {",\n    ".join(map(", ".join, html_hex))}\n}}"
    # Get the length of the array
    html_hex_c_array_len = sum(map(len, html_hex))

    html_file_name, html_file_ext = os.path.splitext(os.path.basename(html_path))
    array_variable_name = f"{html_file_name}_{html_file_ext[1:]}"
    
    # Generate the output string
    output = f"const size_t {array_variable_name}_len = {html_hex_c_array_len};\n" \
             f"const char {array_variable_name}[] PROGMEM = {html_hex_c_array};"

    with open(h_path, "w") as h_file:
        h_file.write(output)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Error: Incorrect number of arguments\nUsage: python {os.path.basename(__file__)} html_path h_path")
        exit(1)

    main(sys.argv[1], sys.argv[2])