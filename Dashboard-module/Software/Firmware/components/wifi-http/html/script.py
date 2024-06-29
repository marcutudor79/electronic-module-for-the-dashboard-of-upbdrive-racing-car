with open("UPBDRIVE_Logo_Horizontal.jpg", "rb") as image_file:
    binary_array = image_file.read()
    array_str = ','.join(str(b) for b in binary_array)

header_content = f'''\
#ifndef IMAGE_H
#define IMAGE_H

const unsigned char image[] = {{{array_str}}};
const unsigned int image_len = {len(binary_array)};

#endif // IMAGE_H
'''

with open("image.h", "w") as header_file:
    header_file.write(header_content)

print("image.h file created successfully.")