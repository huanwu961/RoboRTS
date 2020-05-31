import BT
import rospy
from Blackboard import Blackboard
import PyConfig
import Actions
import Conditions

#rospy.init_node("Demo_decision")

#blackboard  = Blackboard()
def createNode():
    enableshoot = Actions.EnableShoot("enable_shoot")
    disableshoot= Actions.DisableShoot("disable_shoot")
    goto_bloodbuffer = Actions.GOTO_BloodBuffer("goto_bloodbuffer")
    goto_bulletbuffer = Actions.GOTO_BulletBuffer("goto_bulletbuffer")
    random_twist = Actions.RandomTwist("random_twist")
    goto_furthestdefence = Actions.GOTO_FurthestDefence("goto_furthestdefence")
    patrol = Actions.Patrol("patrol")
    chase_enemy = Actions.ChaseEnemy("chase_enemy")


    # Conditions
    isBloodgt500    = Conditions.IsBloodGT("is_bloodgt500",500)
    isStarted       = Conditions.IsStart("is_start")
    isBulletgt100   = Conditions.IsBulletGT("is_bulletgt50",50)
    isBlocked       = Conditions.IsBlocked("is_blocked")
    isInrange2m5    = Conditions.IsInRange("is_inrange2m5",2.5)
    isInrange5m     = Conditions.IsInRange("is_inrange5m",5)
    isBloodgtenemy  = Conditions.IsBloodGTEnemy("is_bloodgtenemy")

    sequential_81   = BT.LogicSequential("attack_still: ->",[isBlocked,isInrange2m5,random_twist])
    sequential_82   = BT.LogicSequential("chase_enemy: ->",[isInrange5m,chase_enemy])
    fallback_71     = BT.LogicFallback("attack_chassis: ?",[sequential_81,sequential_82,patrol])

    parallel_61     = BT.LogicParallel("attack_mode: =>",[enableshoot,fallback_71],2)

    sequential_51   = BT.LogicSequential("bullet_attack: ->",[isBulletgt100,parallel_61])

    fallback_61     = BT.LogicFallback("avoid_chassis: ?",[goto_bulletbuffer,goto_furthestdefence])
    parallel_51     = BT.LogicParallel("avoid_enemy: =>",[disableshoot,fallback_61],2)

    fallback_41     = BT.LogicFallback("enough_blood: ?",[sequential_51,parallel_51])

    sequential_31   = BT.LogicSequential("check_blood: ->",[isBloodgt500,fallback_41])

    sequential_83   = BT.LogicSequential("attack_still:->",[isBlocked,isInrange2m5,random_twist])
    sequential_84   = BT.LogicSequential("chase_enemy: ->",[isInrange5m,chase_enemy])
    fallback_72     = BT.LogicFallback("attack_chassis: ?",[sequential_83,sequential_84,patrol])
    parallel_62     = BT.LogicParallel("attack_mode: =>",[enableshoot,fallback_72],2)
    sequential_52   = BT.LogicSequential("bullet attack: ->",[isBulletgt100,parallel_62])
    fallback_62     = BT.LogicFallback("avoid_chassis: ->",[goto_furthestdefence,goto_bulletbuffer])
    parallel_52     = BT.LogicParallel("avoid_enemy: =>",[disableshoot,fallback_62],2)
    fallback_42     = BT.LogicFallback("greater_blood: ?",[sequential_52,parallel_52])
    sequential_32   = BT.LogicSequential("compare_blood: ->",[isBloodgtenemy,fallback_42])

    parallel_31     = BT.LogicParallel("struggle: =>",[goto_furthestdefence,enableshoot],2)

    bt_root         = BT.LogicFallback("root_bt: ?",[sequential_31,goto_bloodbuffer,sequential_32,parallel_31])

    game_root       = BT.LogicSequential("game_root: ->",[isStarted,bt_root])
    return game_root
