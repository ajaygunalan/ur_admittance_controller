# Debugging Guide

## VS Code Setup

Create `.vscode/launch.json`:
```json
{
    "version": "0.2.0",
    "configurations": [{
        "name": "Debug Admittance",
        "type": "cppdbg",
        "request": "launch",
        "miDebuggerServerAddress": "localhost:3000",
        "program": "${workspaceFolder}/../../install/ur_admittance_controller/lib/ur_admittance_controller/admittance_node",
        "cwd": "${workspaceFolder}"
    }]
}
```

## Build with Debug Symbols

```bash
cd ~/ros2_ws
colcon build --packages-select ur_admittance_controller --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

## Start Debugging

```bash
# Launch with GDB server
ros2 run --prefix 'gdbserver localhost:3000' ur_admittance_controller admittance_node

# In VS Code:
# - Set breakpoints in C++ code
# - Press F5 to start debugger
# - Use F10 (step over) or F11 (step into)
```

## Terminal Debugging with GDB

### Quick Start - Just Run and See the Error

```bash
# Simplest way - just run with GDB
cd ~/ros2_ws
ros2 run --prefix 'gdb' ur_admittance_controller admittance_node

# GDB starts, type 'run' to start
(gdb) run

# When it crashes, see where
(gdb) backtrace
```

### Level 1: Basic Debugging - Set One Breakpoint

Want to stop at a specific line? Here's how:

```bash
# Start GDB with our node
ros2 run --prefix 'gdb' ur_admittance_controller admittance_node

# Set a breakpoint at the FK error
(gdb) break kinematics.hpp:42

# Run the program
(gdb) run

# When it stops, print a value
(gdb) print fk_result

# Continue running
(gdb) continue
```

### Level 2: Essential Commands - The Fab Four

Now let's learn the four commands you'll use 90% of the time:

```gdb
run         # Start your program
next        # Go to next line (like F10 in VS Code)
print       # Show a variable's value
continue    # Keep running after a breakpoint
```

Example session:
```bash
(gdb) break admittance_computations.cpp:300
(gdb) run
# Stops at line 300
(gdb) print kdl_chain_.getNrOfSegments()
$1 = 7
(gdb) next
# Now at line 301
(gdb) continue
```

### Level 3: Stepping Through Code

Three ways to move through your code:

```gdb
next     # Execute current line, don't go into functions
step     # Go INTO the function being called
finish   # Run until current function returns
```

Example - debugging our FK error:
```bash
(gdb) break get_X_BP_current
(gdb) run
# Stops at function start
(gdb) next      # Skip over simple lines
(gdb) next      # Until you reach line 132
(gdb) step      # Step INTO computeForwardKinematics
```

### Level 4: Inspecting Complex Objects

For our KDL chain debugging:

```bash
# Make output pretty
(gdb) set print pretty on

# Print entire objects
(gdb) print components
(gdb) print kdl_chain_

# Call member functions
(gdb) print kdl_chain_.getNrOfSegments()
(gdb) print fk_pos_solver_->chain.getNrOfSegments()

# Show all local variables
(gdb) info locals
```

### Level 5: Advanced - Multiple Breakpoints and Conditions

```bash
# Set multiple breakpoints
(gdb) break admittance_computations.cpp:298  # Before move
(gdb) break admittance_computations.cpp:304  # After move

# Conditional breakpoint
(gdb) break kinematics.hpp:42 if fk_result < 0

# See all breakpoints
(gdb) info breakpoints

# Delete breakpoint 2
(gdb) delete 2
```

### Level 6: Complete Debug Session for KDL Error -4

Let's put it all together:

```bash
# 1. Build with debug symbols
cd ~/ros2_ws
colcon build --packages-select ur_admittance_controller --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash

# 2. Start debugging
ros2 run --prefix 'gdb' ur_admittance_controller admittance_node

# 3. Set strategic breakpoints
(gdb) break admittance_computations.cpp:298   # Before std::move
(gdb) break admittance_computations.cpp:304   # After std::move
(gdb) break kinematics.hpp:42                 # Where error occurs

# 4. Run and inspect
(gdb) run

# At first breakpoint (line 298)
(gdb) print components.robot_chain.getNrOfSegments()
$1 = 7
(gdb) print components.fk_solver->chain.getNrOfSegments() 
$2 = 7

# Continue to after moves
(gdb) continue

# At second breakpoint (line 304)
(gdb) print kdl_chain_.getNrOfSegments()
$3 = 7
(gdb) print fk_pos_solver_->chain.getNrOfSegments()
$4 = 0    # <-- Problem found! Chain is empty in solver

# Continue to see the error
(gdb) continue
# FK failed: KDL error=-4
```

### Pro Tips

- **Repeat last command**: Just press Enter
- **Tab completion**: Type part of variable name and press Tab
- **Interrupt running program**: Ctrl+C
- **Exit GDB**: `quit` or Ctrl+D
- **Command shortcuts**: `b` for break, `p` for print, `c` for continue
- **See source code**: `list` shows code around current line

### Quick Reference Card

| What you want | Command | Example |
|--------------|---------|---------|
| Start debugging | `gdb --args ros2 run pkg node` | `gdb --args ros2 run ur_admittance_controller admittance_node` |
| Run program | `run` | `(gdb) run` |
| Set breakpoint | `break file:line` | `(gdb) break admittance_computations.cpp:300` |
| Next line | `next` | `(gdb) next` |
| Step into | `step` | `(gdb) step` |
| Print value | `print variable` | `(gdb) print kdl_chain_.getNrOfSegments()` |
| Continue | `continue` | `(gdb) continue` |
| See stack | `backtrace` | `(gdb) backtrace` |
| Quit | `quit` | `(gdb) quit` |