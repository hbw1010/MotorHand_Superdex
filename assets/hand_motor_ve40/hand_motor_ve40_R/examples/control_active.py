"""Offline kinematics example: commands only active motors, never hardware."""
from pathlib import Path
import argparse,json,sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
from solve_closed_chain import ClosureSolver

def main():
    hand=ClosureSolver(root/'hand_tree_only.urdf',root/'config/loop_closures.yaml')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint',choices=hand.independent,help='One of the 12 active motor joints; omit to list them')
    parser.add_argument('--angle',type=float,default=0.1,help='Motor shaft angle in radians, using the V1.4-aligned sign convention')
    args=parser.parse_args()
    if args.joint is None:
        print(json.dumps(dict(commandable_joints=hand.independent,angle_unit='rad'),indent=2));return
    all_positions=hand.solve({args.joint:args.angle})
    # Send all_positions to your URDF viewer so passive rods move as well.
    # Only these 12 values belong in an active-motor command/state interface.
    active_positions={n:all_positions[n] for n in hand.independent}
    print(json.dumps(dict(active_positions=active_positions,passive_joints_solved=len(hand.dependent),max_closure_gap_m=hand.max_gap(all_positions)),indent=2))

if __name__=='__main__':main()
