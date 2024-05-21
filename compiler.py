#!/opt/homebrew/bin/python3

import sys
import string
import random


class Interpreter():
    def __init__(self, program_filepath) -> None:
        self.program_filepath: str = program_filepath
        self.line_number: int = 0 

        self.data = {
            'NODE'      : ''.join(random.choices(string.ascii_letters + string.digits, k=10)),
            'PUBLISHER' : [],
            'SUBSCRIBER': [],
            'CLIENT'    : [],
            'SERVER'    : [],
            'ACLIENT'   : [],
            'ASERVER'   : [],
            'TYPE'      : [],
        }

        self.read()
        self.parse()
        self.write()


    def read(self) -> None:
        print('[*] reading')
        self.program_lines = []
        with open(self.program_filepath, 'r') as program_file:
            self.program_lines = [
                line.strip() for line in program_file.readlines()
            ]


    def parse(self) -> None:
        print('[*] parsing')
        func = self.analyze_config
        for line in self.program_lines:
            self.line_number += 1
            line = self.remove_comment(line=line)

            if(line[:-1] == 'PUBLISHER'):
                func = self.analyze_pub
                continue
            elif(line[:-1] == 'SUBSCRIBER'):
                func = self.analyze_sub
                continue
            elif(line[:-1] == 'CLIENT'):
                func = self.analyze_cli
                continue
            elif(line[:-1] == 'SERVER'):
                func = self.analyze_ser
                continue
            elif(':' in line):
                func = self.analyze_config

            func(line=line)

        print(self.data)


    def remove_comment(self, line) -> str:
        index = line.find('#')
        return line if index == -1 else line[0:line.find('#') - 1].strip()


    def analyze_config(self, line) -> None:
        opcode: list[str] = [x.strip() for x in line.split(':')]
        try:
            self.data[opcode[0].upper()] = opcode[1]
        except IndexError:
            return


    def analyze_pub(self, line) -> None:
        args: list[str] = [x.strip() for x in line.split(',')]
        if (args[0] == ''): return
        
        try:
            msg_type    = str(args[0])
            topic       = str(args[1])
        except IndexError:
            print(f"[!] line {self.line_number}: wrong arguments for publisher")
        
        try:
            queue_size  = int(args[2])
        except (IndexError, ValueError):
            queue_size  = 10 

        data = {
            'type'      : msg_type,
            'topic'     : topic,
            'queue_size': queue_size
        }
        self.data['PUBLISHER'].append(data)


    def analyze_sub(self, line) -> None:
        args: list[str] = [x.strip() for x in line.split(',')]
        if (args[0] == ''): return
        
        try:
            msg_type    = str(args[0])
            topic       = str(args[1])
            callback    = str(args[2])
        except IndexError:
            print(f"[!] line {self.line_number}: wrong arguments for subscriber")
        
        try:
            queue_size  = int(args[3])
        except (IndexError, ValueError):
            queue_size  = 10 

        data = {
            'type'      : msg_type,
            'topic'     : topic,
            'callback'  : callback,
            'queue_size': queue_size
        }
        self.data['SUBSCRIBER'].append(data)


    def analyze_cli(self, line) -> None:
        pass


    def analyze_ser(self, line) -> None:
        pass


    def write(self) -> None:
        print('[*] writing')
        with open(f'{self.data['NODE'].lower()}.py', 'w') as output_file:
            output_file.write(
                'import rclpy\n'
                'from rclpy.node import Node\n'
            )



def main():
    program_filepath = sys.argv[1]
    Interpreter(program_filepath)


if __name__ == '__main__':
    main()