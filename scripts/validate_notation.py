#!/usr/bin/env python3
"""
Frame Notation Validation Script
This script verifies that the UR Admittance Controller implementation follows
Drake's coordinate frame notation principles consistently.
"""
import os
import re
import sys
from pathlib import Path
def check_file_for_violations(filepath, patterns):
    """Check a single file for notation violations."""
    violations = []
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
            lines = content.split('\n')
            
            for i, line in enumerate(lines, 1):
                if line.strip().startswith('
                    continue
                    
                for pattern_name, pattern in patterns.items():
                    if re.search(pattern, line):
                        violations.append({
                            'file': filepath,
                            'line': i,
                            'content': line.strip(),
                            'violation': pattern_name
                        })
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
    
    return violations
def validate_frame_notation():
    """Main validation function."""
    
    violation_patterns = {
        'old_transform_cache_ft': r'\bft_transform_cache_\b',
        'old_transform_cache_ee': r'\bee_transform_cache_\b', 
        'old_current_pose': r'\bcurrent_pose_\b',
        'old_desired_pose': r'\bdesired_pose_\b',
        'old_cart_twist': r'\bcart_twist_\b',
        'old_pose_error': r'\bpose_error_\b',
        'old_wrench_raw': r'\bwrench_\s*[^f]',
        'old_method_computePoseError': r'\bcomputePoseError\(\)',
        'ambiguous_transform': r'transform_[^_\s]+$',
    }
    
    required_patterns = {
        'new_transform_base_ft': r'\btransform_base_ft_\b',
        'new_transform_base_tip': r'\btransform_base_tip_\b',
        'new_X_base_tip_current': r'\bX_base_tip_current_\b',
        'new_X_base_tip_desired': r'\bX_base_tip_desired_\b',
        'new_V_base_tip_base': r'\bV_base_tip_base_\b',
        'new_error_tip_base': r'\berror_tip_base_\b',
        'new_F_sensor_base': r'\bF_sensor_base_\b',
        'new_method_computePoseError_tip_base': r'\bcomputePoseError_tip_base\b',
    }
    
    script_dir = Path(__file__).parent
    src_dir = script_dir.parent / 'src'
    include_dir = script_dir.parent / 'include'
    
    print("🔍 Validating Frame Notation Implementation...")
    print("=" * 60)
    
    violations_found = []
    patterns_found = {pattern: False for pattern in required_patterns.keys()}
    
    file_patterns = ['*.cpp', '*.hpp']
    search_dirs = [src_dir, include_dir]
    
    for search_dir in search_dirs:
        if not search_dir.exists():
            print(f"⚠️  Directory {search_dir} not found")
            continue
            
        for pattern in file_patterns:
            for filepath in search_dir.glob(pattern):
                print(f"Checking {filepath.name}...", end=' ')
                
                file_violations = check_file_for_violations(filepath, violation_patterns)
                violations_found.extend(file_violations)
                
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        content = f.read()
                        for pattern_name, pattern in required_patterns.items():
                            if re.search(pattern, content):
                                patterns_found[pattern_name] = True
                except Exception as e:
                    print(f"❌ Error: {e}")
                    continue
                
                if file_violations:
                    print(f"❌ {len(file_violations)} violations")
                else:
                    print("✅ OK")
    
    print("\n" + "=" * 60)
    print("📊 VALIDATION RESULTS")
    print("=" * 60)
    
    if violations_found:
        print(f"❌ VIOLATIONS FOUND: {len(violations_found)}")
        print("\nViolation Details:")
        for violation in violations_found:
            print(f"  📁 {violation['file'].name}:{violation['line']}")
            print(f"     🚫 {violation['violation']}: {violation['content']}")
            print()
    else:
        print("✅ NO VIOLATIONS FOUND - All old notation removed!")
    
    print("\n📋 Required Pattern Coverage:")
    missing_patterns = []
    for pattern_name, found in patterns_found.items():
        status = "✅ Found" if found else "❌ Missing"
        print(f"  {status}: {pattern_name}")
        if not found:
            missing_patterns.append(pattern_name)
    
    print("\n" + "=" * 60)
    if violations_found or missing_patterns:
        print("❌ VALIDATION FAILED")
        if violations_found:
            print(f"   • {len(violations_found)} notation violations found")
        if missing_patterns:
            print(f"   • {len(missing_patterns)} required patterns missing")
        return False
    else:
        print("✅ VALIDATION PASSED")
        print("   • No old notation violations")
        print("   • All required patterns found")
        print("   • Frame notation implementation is complete!")
        return True
if __name__ == "__main__":
    success = validate_frame_notation()
    sys.exit(0 if success else 1)
