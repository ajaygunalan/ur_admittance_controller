# Task Completion Checklist

When completing a task in this project, follow these steps:

## 1. Code Quality Checks
- [ ] Ensure code follows naming conventions (camelCase for functions, snake_case for variables)
- [ ] Add appropriate documentation comments (Doxygen style)
- [ ] Use Result<T> pattern for error handling where appropriate
- [ ] Check for const correctness
- [ ] Verify includes are properly organized

## 2. Build Verification
```bash
# Clean build to catch any issues
cd ~/ros2_ws && rm -rf build/ur_admittance_controller install/ur_admittance_controller
cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller && source install/setup.bash
```

## 3. Runtime Testing
- [ ] Test with simulation if changes affect runtime behavior
- [ ] Verify no runtime errors in logs
- [ ] Check that existing functionality still works

## 4. Documentation
- [ ] Update README.md if adding new features or changing usage
- [ ] Update CLAUDE.md if changing development workflow
- [ ] Add inline comments for complex logic
- [ ] Update relevant header documentation

## 5. Pre-commit Checks
Since no automated tools are configured, manually verify:
- [ ] No compilation warnings
- [ ] No obvious code style violations
- [ ] All TODOs are tracked or resolved
- [ ] No debugging code left in (excessive logging, test code)

## 6. Git Hygiene
- [ ] Stage only relevant files
- [ ] Write clear commit message following convention
- [ ] Ensure no large binary files or generated files are committed

## Note on Missing Tools
This project currently lacks:
- Automated formatting (no .clang-format)
- Linting configuration
- Unit tests implementation
- CI/CD pipeline

Consider adding these in future improvements.