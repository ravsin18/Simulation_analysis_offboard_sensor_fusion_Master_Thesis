#!/usr/bin/env python3
import sys
import os

def main():
    print("="*60)
    print("--- ROS 2 PYTHON ENVIRONMENT DIAGNOSTIC ---")

    print(f"\n[1] Python Executable Path:")
    print(f"    {sys.executable}")

    print(f"\n[2] Python Version:")
    print(f"    {sys.version}")

    print(f"\n[3] Python Search Path (sys.path):")
    for i, path in enumerate(sys.path):
        print(f"    {i}: {path}")

    print("\n--- END OF DIAGNOSTIC ---")
    print("="*60)

if __name__ == '__main__':
    main()