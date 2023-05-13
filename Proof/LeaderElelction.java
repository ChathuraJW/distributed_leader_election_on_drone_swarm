import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Random;

public class LeaderElelction {
    private static final int TOTAL_NODE_COUNT = 3;
    private static final List<Integer> RSV_VECTOR = new ArrayList<>();
    private static final int HEARTBEAT_INTERVAL = 1000;// 2500;
    private static final int WAIT_TIME_TO_START_ELECTION = HEARTBEAT_INTERVAL * 1;
    private static final int WAIT_TIME = HEARTBEAT_INTERVAL;
    private static final double LFR = 1.0;
    private static final double MESSAGE_LOSS_PROBABILITY = 0.0;

    public static class DroneModel {
        private Object[][] rsvVector;
        private Object[][] voteVector;
        private DroneModel[] allNodes;
        private int nodeId;
        private int leaderId;
        private int myRsv;
        private long lastHbTime;
        private boolean isLeaderAlive;

        public DroneModel(int nodeId, int leaderId) {
            this.rsvVector = new Object[TOTAL_NODE_COUNT][2];
            this.voteVector = new Object[TOTAL_NODE_COUNT][2];
            this.allNodes = null;
            this.nodeId = nodeId;
            this.leaderId = leaderId;
            this.myRsv = 0;
            this.lastHbTime = System.currentTimeMillis();
            this.isLeaderAlive = true;
        }

        public void setNodeObjList(DroneModel[] nodeObjects) {
            this.allNodes = nodeObjects;
        }

        public void communicate(int[] message) {
            switch (message[0]) {
                case 0: // heartbeat message [0, leader_id, rsv_val]
                    this.isLeaderAlive = true;
                    this.leaderId = message[1];
                    this.lastHbTime = System.currentTimeMillis();
                    break;
                case 1: // election reply [1, node_id, rsv_val, cal_time]
                    this.rsvVector[message[1] - 1] = new Object[] {
                            message[1], message[2]
                    };
                    break;
                case 2: // inform winner [2, node_id, confidence_score]
                    this.voteVector[message[1] - 1] = new Object[] {
                            message[1], message[2]
                    };
                    break;
                case 3: // leader's broadcast [3, leader_id]
                    this.leaderId = message[1];
                    this.isLeaderAlive = true;
                    break;
                default:
                    break;
            }
        }

        public void broadcast(int[] message) {
            System.out.println("Node with ID: " + this.nodeId + " broadcast message: " + Arrays.toString(message));
            // Random random = new Random();
            for (int i = 0; i < TOTAL_NODE_COUNT; i++) {
                // if (random.nextDouble() > MESSAGE_LOSS_PROBABILITY) {
                allNodes[i].communicate(message);
                // }
            }
        }

        public void leaderWork() {
            while (true) {
                if (new Random().nextDouble() > LFR) {
                    // leader alive, make broadcast
                    // get RSV from pre-defined vector
                    this.myRsv = RSV_VECTOR.get(this.nodeId - 1);
                    int[] hbMessage = {
                            0,
                            this.nodeId,
                            this.myRsv
                    };
                    this.broadcast(hbMessage);
                    try {
                        Thread.sleep(HEARTBEAT_INTERVAL);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                } else {
                    // leader failed
                    System.out.println("Leader failed");
                    break;
                }
            }
        }

        public void followerWork() {
            while (true) {
                if (this.lastHbTime + WAIT_TIME_TO_START_ELECTION > System.currentTimeMillis()) {
                    // normal operation
                    System.out.println("Node with ID: " + this.nodeId + " move according to leader, with node ID: "
                            + this.leaderId + ".\n");
                } else {
                    System.out.println("Node with ID: " + this.nodeId + " start election.\n");
                    // leader failed, do election
                    this.isLeaderAlive = false;
                    // ------------------------ Round 1 ------------------------
                    long timerR1 = System.currentTimeMillis();
                    this.myRsv = RSV_VECTOR.get(this.nodeId - 1);
                    int[] erMessage = { 1, this.nodeId, this.myRsv, (int) (System.currentTimeMillis() / 1000) };
                    this.broadcast(erMessage);
                    // wait until round 1 finish
                    while (timerR1 + WAIT_TIME < System.currentTimeMillis()) {
                        try {
                            Thread.sleep(1);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                    // ------------------------ Round 2 ------------------------
                    long timerR2 = System.currentTimeMillis();
                    // Arrays.sort(this.rsvVector, Comparator.comparingInt(a -> (Integer) a[1]));
                    // Arrays.sort(this.rsvVector, Comparator
                    // .comparingInt(a -> Optional.ofNullable(a).map(arr -> (Integer)
                    // arr[1]).orElse(0)));
                    // int mySelectionNodeId = (int) this.rsvVector[this.rsvVector.length - 1][0];

                    // sort and get max value having elelement in array
                    int mySelectionNodeId = 0;
                    int max_rsv = 0;
                    for (int i = this.rsvVector.length - 1; i >= 0; i--) {
                        if (this.rsvVector[i][0] == null) {
                            continue;
                        }
                        if ((Integer) this.rsvVector[i][1] > max_rsv) {
                            mySelectionNodeId = (Integer) this.rsvVector[i][0];
                            max_rsv = (Integer) this.rsvVector[i][1];
                        }
                    }

                    int confidenceValue = (int) (this.rsvVector.length / (TOTAL_NODE_COUNT - 1)) * 100;
                    int[] infoMessage = { 2, mySelectionNodeId, confidenceValue };
                    this.broadcast(infoMessage);
                    while (timerR2 + WAIT_TIME < System.currentTimeMillis()) {
                        try {
                            Thread.sleep(1);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                    // ------------------------ Round 3 ------------------------
                    long timerR3 = System.currentTimeMillis();

                    int[][] voteCounter = new int[TOTAL_NODE_COUNT][2];
                    for (Object[] vote : this.voteVector) {
                        if (vote[0] == null)
                            continue;
                        voteCounter[(int) vote[0] - 1][0] = (int) vote[0];
                        voteCounter[(int) vote[0] - 1][1] += (int) vote[1];
                    }

                    // Arrays.sort(voteCounter, Comparator.comparingInt(a -> a[1]));
                    // Arrays.sort(voteCounter, Comparator
                    // .comparingInt(a -> Optional.ofNullable(a).map(arr -> (Integer)
                    // arr[1]).orElse(0)));
                    // int selectedNewLeaderId = voteCounter[voteCounter.length - 1][0];

                    int selectedNewLeaderId = 0;
                    int max_vote = 0;
                    for (int i = voteCounter.length - 1; i >= 0; i--) {
                        if (voteCounter[i] == null) {
                            continue;
                        }
                        if ((Integer) voteCounter[i][1] > max_vote) {
                            selectedNewLeaderId = (Integer) voteCounter[i][0];
                            max_vote = (Integer) voteCounter[i][1];
                        }
                    }

                    if (selectedNewLeaderId == this.nodeId) {
                        // I am the leader
                        int[] lbMessage = { 3, selectedNewLeaderId };
                        this.broadcast(lbMessage);
                    }
                    while (timerR3 + WAIT_TIME < System.currentTimeMillis() && this.isLeaderAlive == false) {
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    System.out.println("I am node with ID: " + this.nodeId + ". And I know node with ID: "
                            + this.leaderId + " is selected as the leader.(end of election))");
                    break;
                }
            }
        }

        public void doWork() {
            if (this.nodeId == this.leaderId) {
                // I am the leader, do leader's work
                new Thread(this::leaderWork).start();
            } else {
                // I am a follower, do follower's work
                new Thread(this::followerWork).start();
            }
        }

    }

    public static void main(String[] args) {
        // fill RSV vector with unique values
        Random random = new Random();
        while (RSV_VECTOR.size() < TOTAL_NODE_COUNT) {
            int val = random.nextInt(101);
            if (!RSV_VECTOR.contains(val)) {
                RSV_VECTOR.add(val);
            }
        }

        ArrayList<DroneModel> allNodes = new ArrayList<DroneModel>();

        for (int i = 0; i < TOTAL_NODE_COUNT; i++) {
            DroneModel node = new DroneModel(i + 1, 1);
            allNodes.add(node);

        }

        DroneModel[] nodeArray = new DroneModel[allNodes.size()];
        for (int i = 0; i < allNodes.size(); i++) {
            nodeArray[i] = allNodes.get(i);
        }

        for (int i = 0; i < TOTAL_NODE_COUNT; i++) {
            nodeArray[i].setNodeObjList(nodeArray);
            nodeArray[i].doWork();
        }
    }

}