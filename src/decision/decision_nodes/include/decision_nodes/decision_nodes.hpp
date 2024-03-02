#include "behaviortree_cpp/bt_factory.h"

#include "decision_nodes/game_over_node.h"
#include "decision_nodes/beginning_state_node.h"
#include "decision_nodes/simple_condition_node.h"
#include "decision_nodes/see_enemy_node.h"

#include "decision_nodes/simple_action_node.h"

#include "decision_nodes/move_base_node.h"
#include "decision_nodes/wait_for_seconds_node.h"
#include "decision_nodes/near_position_node.h"
#include "decision_nodes/trace_enemy.h"

#include "decision_nodes/find_enemy.h"

#include "decision_nodes/_switch_node.h"

//注册行为树节点
inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    /*判断节点*/
    //ros正常运行
    factory.registerSimpleCondition("RosOk", std::bind(RosOk));
    //游戏结束
    factory.registerSimpleCondition("IsGameOver", std::bind(IsGameOver));
    //开局阶段
    factory.registerSimpleCondition("IsBeginningState", std::bind(IsBeginningState));
    //位置过半
    factory.registerSimpleCondition("PositionOverHalf", std::bind(PositionOverHalf));
    //敌方哨兵在左边
    factory.registerSimpleCondition("EnemyGuardCome", std::bind(EnemyGuardCome));
    //敌方哨兵在前方
    factory.registerSimpleCondition("EnemyGuardDefend", std::bind(EnemyGuardDefend));
    //敌方哨兵残血
    factory.registerSimpleCondition("GuardHurt", std::bind(GuardHurt));
    //劣势
    factory.registerSimpleCondition("InferiorSituation", std::bind(InferiorSituation));
    //队友在附近
    factory.registerSimpleCondition("AllyStayBy", std::bind(AllyStayBy));
    //敌方哨兵阵亡
    factory.registerSimpleCondition("EnemyGuardKilled", std::bind(EnemyGuardKilled));
    //敌方攻击慢
    factory.registerSimpleCondition("EnemyAttackSlowly", std::bind(EnemyAttackSlowly));
    //血量不足
    factory.registerSimpleCondition("LowBlood", std::bind(LowBlood));
    //敌方哨兵消失
    factory.registerSimpleCondition("EnemyGuardDisappear", std::bind(EnemyGuardDisappear));
    //敌方哨兵在我方区域
    factory.registerSimpleCondition("EnemyGuardInMyDomain", std::bind(EnemyGuardInMyDomain));
    //敌方哨兵在敌方区域
    factory.registerSimpleCondition("EnemyGuardInEnemyDomain", std::bind(EnemyGuardInEnemyDomain));
    //敌方哨兵位置未知
    factory.registerSimpleCondition("EnemyGuardInUnknowDomain", std::bind(EnemyGuardInUnknowDomain));
    //敌方哨兵从左边撤退
    factory.registerSimpleCondition("EnemyGuardRetraitL", std::bind(EnemyGuardRetraitL));
    //攻击效率低
    factory.registerSimpleCondition("LowEfficience", std::bind(LowEfficience));
    //敌方哨兵在前方撤退
    factory.registerSimpleCondition("EnemyGuardRetraitR", std::bind(EnemyGuardRetraiR));
    //友方状态差 血量小于150
    factory.registerSimpleCondition("AllyPoorState", std::bind(AllyPoorState));
    //血量高于200
    factory.registerSimpleCondition("BloodOver200", std::bind(BloodOver200));
    //状态锁
    factory.registerSimpleCondition("StateLocked", std::bind(StateLocked));
    //攻击中
    factory.registerSimpleCondition("Attacking", std::bind(Attacking));
    //基地被攻击
    factory.registerSimpleCondition("BaseBeenAttacked", std::bind(BaseBeenAttacked));
    //占领增益区
    factory.registerSimpleCondition("WhtherOccupyBenefit", std::bind(WhtherOccupyBenefit));
    //血量低于250
    factory.registerSimpleCondition("BloodLess200", std::bind(BloodLess200));
    //血量恢复完成
    factory.registerSimpleCondition("RecoverFinished", std::bind(RecoverFinished));
    //优势
    factory.registerSimpleCondition("FavorableSituation", std::bind(FavorableSituation));
    //敌人全部阵亡 EnemyAllKilled
    //比赛时间不足1分钟 TimeLessOneMin

    //发现敌方
    factory.registerNodeType<FindEnemy>("FindEnemy");
    
    /*简单执行节点*/
    //锁住状态
    factory.registerSimpleAction("LockState", std::bind(LockState));
    //结束开局状态
    factory.registerSimpleAction("EndBeginningState", std::bind(EndBeginningState));
    
    /*执行节点*/
    factory.registerNodeType<MoveBaseNode>("MoveBase");
    factory.registerNodeType<WaitForSecondsNode>("WaitForSeconds");
    factory.registerNodeType<NearPositionNode>("NearPosition");
    factory.registerNodeType<TraceEnemy>("TraceEnemy");
}