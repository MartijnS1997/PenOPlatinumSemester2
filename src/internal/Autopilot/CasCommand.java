package internal.Autopilot;

/**
 * The commands that can be issued by the collision detection system
 * ASCEND: the drone must ascend
 * DESCEND: the drone must descend
 * MAINTAIN_ALTITUDE: the drone must maintain its current altitude
 * NOP: the drone may continue normal operation
 */
enum CasCommand{
    ASCEND,DESCEND, NOP
}