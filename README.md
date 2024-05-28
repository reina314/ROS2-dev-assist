# ROS2-dev-assist

## Setup
1. Clone the repository
```bash
git clone https://github.com/reina314/ROS2-dev-assist.git
```
2. Run a setup script inside the repo<br>
It creates a symbolic link of the `compiler.py` in your path as `pycompile`.
```bash
bash ROS2-dev-assist/setup.bash
```
3. Done!

## How to Use
1. Copy `template.yaml` into a ROS's Python package.
2. Configure them as needed. See examples in `template.yaml` for more details. Take note that each node must have its own yaml config file.
3. Run the following command in the directory where the config files are located.
   ```bash
    pycompile node1.yaml node2.yaml
   ```
4. Done!

If you make any changes in the config files after they are compiled, run again the command above to apply them. The compiler only takes care of newly added configs and never deletes any deducted contents, so you have to manually delete the unnecessary part of the code.