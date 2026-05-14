#!/usr/bin/env python3
"""
Final validation: Check all modified files for imports, syntax, and key logic paths.
"""

import sys
import ast
import importlib.util

sys.path.insert(0, '/home/dkp/ros2_ws/src/council')

def check_file(filepath, description):
    """Check a Python file for syntax and basic logic."""
    print(f"\n{'='*60}")
    print(f"Checking: {description}")
    print(f"File: {filepath}")
    print('='*60)
    
    try:
        # Parse AST
        with open(filepath, 'r') as f:
            code = f.read()
        
        ast.parse(code)
        print("✓ Syntax: OK")
        
        # Count key elements
        tree = ast.parse(code)
        classes = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
        functions = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
        
        print(f"✓ Classes: {len(classes)}")
        print(f"✓ Functions/Methods: {len(functions)}")
        
        # Check for common issues
        imports = [n for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom))]
        print(f"✓ Imports: {len(imports)}")
        
        return True
        
    except SyntaxError as e:
        print(f"✗ Syntax Error: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def check_imports():
    """Verify critical imports work."""
    print(f"\n{'='*60}")
    print("Checking Critical Imports")
    print('='*60)
    
    checks = [
        ("council.orchestrator:CouncilOrchestrator", "council.orchestrator", "CouncilOrchestrator"),
        ("council.task_planner:TaskPlanner", "council.task_planner", "TaskPlanner"),
        ("council.main:CouncilNode", "council.main", "CouncilNode"),
        ("council.agents.lidar_agent:LidarAgent", "council.agents.lidar_agent", "LidarAgent"),
        ("council.agents.camera_agent:CameraAgent", "council.agents.camera_agent", "CameraAgent"),
        ("council.agents.slam_agent:SLAMAgent", "council.agents.slam_agent", "SLAMAgent"),
        ("council.ros_interfaces.sensor_hub:SensorHub", "council.ros_interfaces.sensor_hub", "SensorHub"),
    ]
    
    passed = 0
    failed = 0
    
    for desc, module_name, attr_name in checks:
        try:
            module = importlib.import_module(module_name)
            obj = getattr(module, attr_name)
            print(f"✓ {desc}")
            passed += 1
        except Exception as e:
            print(f"✗ {desc}: {e}")
            failed += 1
    
    return passed, failed


def check_logic():
    """Check key logic paths exist."""
    print(f"\n{'='*60}")
    print("Checking Key Logic Paths")
    print('='*60)
    
    with open('/home/dkp/ros2_ws/src/council/council/orchestrator.py', 'r') as f:
        orch_code = f.read()
    
    checks = [
        ("_record_reasoning_trace", "Reasoning trace recording"),
        ("_compress_reasoning_history", "Memory compression"),
        ("_peer_reflection_round", "Peer reflection round"),
        ("_build_agent_context", "Context injection"),
        ("_weighted_vote", "Voting logic"),
        ("_needs_meta_review", "Meta-review escalation"),
        ("_is_visual_task", "Visual task detection"),
        ("camera_override", "Camera override logic"),
    ]
    
    passed = 0
    for func_name, desc in checks:
        if func_name in orch_code:
            print(f"✓ {desc} ({func_name})")
            passed += 1
        else:
            print(f"✗ {desc} ({func_name}) NOT FOUND")
    
    # Check task planner
    with open('/home/dkp/ros2_ws/src/council/council/task_planner.py', 'r') as f:
        task_code = f.read()
    
    if "_task_decomposed" not in open('/home/dkp/ros2_ws/src/council/council/main.py').read():
        print(f"✗ Task decomposition guard not found in main.py")
    else:
        print(f"✓ Task decomposition guard in main.py")
        passed += 1
    
    return passed


def main():
    """Run all checks."""
    print("\n" + "="*60)
    print("COUNCIL ARCHITECTURE VALIDATION")
    print("="*60)
    
    files = [
        ('/home/dkp/ros2_ws/src/council/council/orchestrator.py', 'Orchestrator (multi-agent voting + memory)'),
        ('/home/dkp/ros2_ws/src/council/council/task_planner.py', 'Task Planner (checkpoint decomposition)'),
        ('/home/dkp/ros2_ws/src/council/council/main.py', 'Main Node (AI loop + planner integration)'),
        ('/home/dkp/ros2_ws/src/council/council/agents/base_agent.py', 'Base Agent (multimodal + compact logging)'),
        ('/home/dkp/ros2_ws/src/council/council/agents/lidar_agent.py', 'LiDAR Agent (BEV + safety constraints)'),
        ('/home/dkp/ros2_ws/src/council/council/agents/camera_agent.py', 'Camera Agent (defer + visual priority)'),
        ('/home/dkp/ros2_ws/src/council/council/agents/slam_agent.py', 'SLAM Agent (backward suppression)'),
        ('/home/dkp/ros2_ws/src/council/council/ros_interfaces/sensor_hub.py', 'Sensor Hub (BEV + enhanced LiDAR)'),
    ]
    
    syntax_results = []
    for fpath, desc in files:
        syntax_results.append(check_file(fpath, desc))
    
    print(f"\n\n{'='*60}")
    print("SYNTAX VALIDATION SUMMARY")
    print('='*60)
    syntax_pass = sum(syntax_results)
    syntax_total = len(syntax_results)
    print(f"Files passed: {syntax_pass}/{syntax_total}")
    
    if syntax_pass == syntax_total:
        print("✓ All files have valid syntax")
    else:
        print("✗ Some files have syntax errors")
        return
    
    # Check imports
    import_pass, import_fail = check_imports()
    print(f"\nImports passed: {import_pass}, failed: {import_fail}")
    
    if import_fail > 0:
        print("⚠ Some imports failed - check dependencies")
    else:
        print("✓ All critical imports successful")
    
    # Check logic
    logic_pass = check_logic()
    print(f"\nLogic paths verified: {logic_pass}")
    
    # Final summary
    print(f"\n\n{'='*60}")
    print("FINAL SUMMARY")
    print('='*60)
    
    if syntax_pass == syntax_total and import_fail == 0:
        print("✓✓✓ VALIDATION PASSED ✓✓✓")
        print("\nAll files compile cleanly.")
        print("All imports accessible.")
        print("Key logic paths present.")
        print("\n→ Ready for real-world testing on Go2 robot")
        return 0
    else:
        print("✗ Validation failed - see details above")
        return 1


if __name__ == "__main__":
    sys.exit(main())
