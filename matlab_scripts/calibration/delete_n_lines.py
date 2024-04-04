import sys


def delete_lines(file_path, n):

    with open(file_path, 'r', encoding='utf-8') as file:
        lines = file.readlines()

# 保留前585行
    with open(file_path, 'w', encoding='utf-8') as file:
        file.writelines(lines[:n])

def main():
    if len(sys.argv) == 3:
        file_path = sys.argv[1]
        num_lines = int(sys.argv[2])
        delete_lines(file_path, num_lines)

if __name__=="__main__":
    main()
