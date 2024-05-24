# ROS2-Coding-Assistant

## How to Use
1. Configure each yaml file. There must be one yaml file per node.
2. Run this command in the directory where the configuration files are located.
   ```bash
    ./compiler.py node1.yaml node2.yaml
   ```
3. Done!

If you make changes in the configuration files after they are compiled, run again the command above to apply the change. The compiler only takes care of those added, and the command has no effect on those that are deleted.