import { useEffect, useRef, useState } from 'react';
import { Box, Typography, Button, Paper, TextField, Snackbar, Alert, Grid } from '@mui/material';
import * as Blockly from 'blockly';
import axios from 'axios';

// [블록리 에디터 컴포넌트] 사용자가 시각적으로 로봇 행동 트리(Behavior Tree)를 설계할 수 있는 도구
export default function BlocklyEditor() {
    const blocklyDiv = useRef<HTMLDivElement>(null);
    const workspaceRef = useRef<Blockly.WorkspaceSvg | null>(null);

    const [filename, setFilename] = useState("advanced_striker");
    const [xmlPreview, setXmlPreview] = useState("");
    const [snackbar, setSnackbar] = useState<{ open: boolean, msg: string, severity: 'success' | 'error' }>({
        open: false, msg: '', severity: 'success'
    });

    const updateXmlPreview = (workspace: Blockly.WorkspaceSvg) => {
        const body = generateBTXml(workspace);
        const fullXml = `<root BTCPP_format="4" main_tree_to_execute="MainTree">\n  <BehaviorTree ID="MainTree">\n${body}  </BehaviorTree>\n</root>`;
        setXmlPreview(fullXml);
    };

    useEffect(() => {
        if (blocklyDiv.current && !workspaceRef.current) {
            defineCustomBlocks();

            workspaceRef.current = Blockly.inject(blocklyDiv.current, {
                toolbox: {
                    "kind": "categoryToolbox",
                    "contents": [
                        {
                            "kind": "category",
                            "name": "Flow Control",
                            "colour": "210",
                            "contents": [
                                { "kind": "block", "type": "bt_sequence" },
                                { "kind": "block", "type": "bt_reactive_sequence" },
                                { "kind": "block", "type": "bt_fallback" },
                                { "kind": "block", "type": "bt_reactive_fallback" },
                                { "kind": "block", "type": "bt_while_do_else" },
                                { "kind": "block", "type": "bt_repeat" },
                            ]
                        },
                        {
                            "kind": "category",
                            "name": "Conditions & Variables",
                            "colour": "120",
                            "contents": [
                                { "kind": "block", "type": "cond_ball_dist" },
                                { "kind": "block", "type": "cond_is_winning" },
                                { "kind": "block", "type": "cond_check_blackboard" },
                                { "kind": "block", "type": "cond_script_condition" },
                                { "kind": "block", "type": "action_script_execution" }
                            ]
                        },
                        {
                            "kind": "category",
                            "name": "Robot Actions",
                            "colour": "20",
                            "contents": [
                                { "kind": "block", "type": "action_kick" },
                                { "kind": "block", "type": "action_chase" },
                                { "kind": "block", "type": "action_adjust" },
                                { "kind": "block", "type": "action_dribble_to_goal" },
                                { "kind": "block", "type": "action_set_velocity" },
                                { "kind": "block", "type": "action_wait" }
                            ]
                        },
                        {
                            "kind": "category",
                            "name": "Strategy Actions",
                            "colour": "290",
                            "contents": [
                                { "kind": "block", "type": "action_self_locate" },
                                { "kind": "block", "type": "action_search_ball" },
                                { "kind": "block", "type": "action_pass_receive" },
                                { "kind": "block", "type": "action_off_the_ball" },
                                { "kind": "block", "type": "action_go_back_in_field" },
                                { "kind": "block", "type": "action_calc_kick_dir" }
                            ]
                        },
                        {
                            "kind": "category",
                            "name": "SubTrees",
                            "colour": "160",
                            "contents": [
                                { "kind": "block", "type": "subtree_cam_find_and_track" },
                                { "kind": "block", "type": "subtree_locate" },
                                { "kind": "block", "type": "subtree_find_ball" },
                                { "kind": "block", "type": "subtree_strategy_tree" }
                            ]
                        }
                    ]
                },
                zoom: {
                    controls: true,
                    wheel: true,
                    startScale: 0.8,
                    maxScale: 3,
                    minScale: 0.3,
                    scaleSpeed: 1.2
                },
                grid: {
                    spacing: 20,
                    length: 3,
                    colour: '#bbb',
                    snap: true
                },
                trashcan: true
            });

            workspaceRef.current.addChangeListener((event) => {
                if (event.type === Blockly.Events.BLOCK_MOVE ||
                    event.type === Blockly.Events.BLOCK_CHANGE ||
                    event.type === Blockly.Events.BLOCK_DELETE ||
                    event.type === Blockly.Events.BLOCK_CREATE) {
                    if (workspaceRef.current) updateXmlPreview(workspaceRef.current);
                }
            });


            const initialXml = `
            <xml xmlns="https://developers.google.com/blockly/xml">
                <!-- 최상단: 반응형 펄스 (무한 반복하며 조건 검사) -->
                <block type="bt_reactive_sequence" x="50" y="50">
                    <statement name="CHILDREN">
                        
                        <!-- 1. 항시 로컬라이제이션 -->
                        <block type="action_self_locate">
                            <next>
                                <!-- 2. 게임 상태에 따른 분기 (Fallback) -->
                                <block type="bt_reactive_fallback">
                                    <statement name="CHILDREN">
                                        
                                        <!-- A. 공을 잃어버렸을 때 (FindBall 서브트리 실행) -->
                                        <block type="bt_sequence">
                                            <statement name="CHILDREN">
                                                <block type="cond_script_condition">
                                                    <field name="CODE">decision == 'find'</field>
                                                </block>
                                                <next>
                                                    <block type="subtree_find_ball"></block>
                                                </next>
                                            </statement>
                                            
                                            <next>
                                                <!-- B. 정규 인플레이 상태 (공이 보일 때) -->
                                                <block type="bt_reactive_sequence">
                                                    <statement name="CHILDREN">
                                                        <!-- 카메라 추적 & 킥 방향 계산 & 전략 트리 융합 -->
                                                        <block type="subtree_cam_find_and_track">
                                                            <next>
                                                                <block type="action_calc_kick_dir">
                                                                    <next>
                                                                        <block type="subtree_strategy_tree">
                                                                            <next>
                                                                                
                                                                                <!-- 실제 행동 분기 (Fallback: 위에서부터 하나씩 시도) -->
                                                                                <block type="bt_fallback">
                                                                                    <statement name="CHILDREN">
                                                                                        
                                                                                        <!-- 1순위: 킥 조건 만족 시 킥! -->
                                                                                        <block type="bt_sequence">
                                                                                            <statement name="CHILDREN">
                                                                                                <block type="cond_script_condition">
                                                                                                    <field name="CODE">decision == 'kick'</field>
                                                                                                </block>
                                                                                                <next>
                                                                                                    <block type="action_kick">
                                                                                                        <field name="KICK_SPEED">1.3</field>
                                                                                                    </block>
                                                                                                </next>
                                                                                            </statement>
                                                                                            
                                                                                            <next>
                                                                                                <!-- 2순위: 드리블 조건 만족 시 골대로 드리블 -->
                                                                                                <block type="bt_sequence">
                                                                                                    <statement name="CHILDREN">
                                                                                                        <block type="cond_script_condition">
                                                                                                            <field name="CODE">decision == 'dribble'</field>
                                                                                                        </block>
                                                                                                        <next>
                                                                                                            <block type="action_dribble_to_goal">
                                                                                                                <field name="DIST">2.0</field>
                                                                                                            </block>
                                                                                                        </next>
                                                                                                    </statement>
                                                                                                    
                                                                                                    <next>
                                                                                                        <!-- 3순위: 기타 상황 (무작정 체이스) -->
                                                                                                        <block type="bt_sequence">
                                                                                                            <statement name="CHILDREN">
                                                                                                                <block type="cond_script_condition">
                                                                                                                    <field name="CODE">decision == 'chase'</field>
                                                                                                                </block>
                                                                                                                <next>
                                                                                                                    <block type="action_chase">
                                                                                                                        <field name="SAFE_DIST">0.5</field>
                                                                                                                    </block>
                                                                                                                </next>
                                                                                                            </statement>
                                                                                                        </block>
                                                                                                    </next>
                                                                                                </block>
                                                                                            </next>
                                                                                        </block>
                                                                                        
                                                                                    </statement>
                                                                                </block>
                                                                                
                                                                            </next>
                                                                        </block>
                                                                    </next>
                                                                </block>
                                                            </next>
                                                        </block>
                                                    </statement>
                                                </block>
                                            </next>
                                        </block>
                                    </statement>
                                </block>
                            </next>
                        </block>
                        
                    </statement>
                </block>
            </xml>`;
            Blockly.Xml.domToWorkspace(Blockly.utils.xml.textToDom(initialXml), workspaceRef.current);
            updateXmlPreview(workspaceRef.current);
        }
    }, []);

    const handleSave = async () => {
        if (!xmlPreview) return;
        try {
            await axios.post('http://localhost:8000/api/strategies', { name: filename, xml: xmlPreview });
            setSnackbar({ open: true, msg: `Saved ${filename}.xml successfully!`, severity: 'success' });
        } catch (e) {
            setSnackbar({ open: true, msg: "Failed to save strategy", severity: 'error' });
        }
    };

    const handleDeploy = async () => {
        if (!xmlPreview) return;
        try {
            await axios.post('http://localhost:8000/api/deploy_strategy', { robot_id: "all", strategy_xml: xmlPreview });
            setSnackbar({ open: true, msg: "Deployed to All Robots!", severity: 'success' });
        } catch (e) {
            setSnackbar({ open: true, msg: "Failed to deploy", severity: 'error' });
        }
    };

    return (
        <Paper sx={{ p: 2, height: 'calc(100vh - 150px)', display: 'flex', flexDirection: 'column', backgroundColor: '#fdfdfd' }}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 2, alignItems: 'center' }}>
                <Typography variant="h5" fontWeight="bold" color="primary">Advanced Behavior Tree Editor</Typography>
                <Box sx={{ display: 'flex', gap: 1 }}>
                    <TextField size="small" label="Strategy Name" value={filename} onChange={(e) => setFilename(e.target.value)} />
                    <Button variant="contained" color="success" onClick={handleSave}>Save XML</Button>
                    <Button variant="contained" color="primary" onClick={handleDeploy}>Deploy to Robots</Button>
                </Box>
            </Box>

            <Grid container spacing={2} sx={{ flexGrow: 1, minHeight: 0 }}>
                {/* 좌측 Blockly */}
                <Grid item xs={12} md={8}>
                    <Box sx={{ height: '100%', border: '2px solid #ddd', borderRadius: 2, overflow: 'hidden', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}>
                        <div ref={blocklyDiv} style={{ height: '100%', width: '100%' }} />
                    </Box>
                </Grid>
                {/* 우측 Live XML */}
                <Grid item xs={12} md={4}>
                    <Box sx={{
                        height: '100%', backgroundColor: '#282c34', color: '#abb2bf', p: 2, borderRadius: 2,
                        overflowY: 'auto', fontFamily: '"Fira Code", monospace', fontSize: '0.9rem', whiteSpace: 'pre-wrap',
                        boxShadow: 'inset 0 0 10px rgba(0,0,0,0.5)'
                    }}>
                        {xmlPreview}
                    </Box>
                </Grid>
            </Grid>

            {/* Error/Success Popup */}
            <Snackbar open={snackbar.open} autoHideDuration={3000} onClose={() => setSnackbar({ ...snackbar, open: false })}>
                <Alert severity={snackbar.severity} sx={{ width: '100%' }}>{snackbar.msg}</Alert>
            </Snackbar>
        </Paper>
    );
}

// ==========================================
// [Blockly Custom Blocks]
// ==========================================
function defineCustomBlocks() {
    // ---- 1. Flow Control ----
    Blockly.Blocks['bt_sequence'] = {
        init: function () {
            this.appendDummyInput().appendField("Sequence (->)");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(210);
        }
    };
    Blockly.Blocks['bt_reactive_sequence'] = {
        init: function () {
            this.appendDummyInput().appendField("ReactiveSequence (=>)");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(210);
        }
    };
    Blockly.Blocks['bt_fallback'] = {
        init: function () {
            this.appendDummyInput().appendField("Fallback (?)");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(210);
        }
    };
    Blockly.Blocks['bt_reactive_fallback'] = {
        init: function () {
            this.appendDummyInput().appendField("ReactiveFallback (!?)");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(210);
        }
    };
    Blockly.Blocks['bt_while_do_else'] = {
        init: function () {
            this.appendDummyInput().appendField("WhileDoElse");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(210);
        }
    };
    Blockly.Blocks['bt_repeat'] = {
        init: function () {
            this.appendDummyInput().appendField("Repeat")
                .appendField(new Blockly.FieldNumber(3), "NUM")
                .appendField("times");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(210);
        }
    };

    // ---- 2. Conditions & Variables ----
    Blockly.Blocks['cond_check_blackboard'] = {
        init: function () {
            this.appendDummyInput()
                .appendField("Check Blackboard")
                .appendField("Key:").appendField(new Blockly.FieldTextInput("target_x"), "KEY")
                .appendField("Value:").appendField(new Blockly.FieldTextInput("0"), "VAL");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(120);
        }
    };
    Blockly.Blocks['cond_script_condition'] = {
        init: function () {
            this.appendDummyInput()
                .appendField("If [Script]:")
                .appendField(new Blockly.FieldTextInput("decision == 'kick'"), "CODE");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(120);
        }
    };
    Blockly.Blocks['action_script_execution'] = {
        init: function () {
            this.appendDummyInput()
                .appendField("Execute [Script]:")
                .appendField(new Blockly.FieldTextInput("target_x = 1.5;"), "CODE");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(120);
        }
    };
    Blockly.Blocks['cond_ball_dist'] = {
        init: function () {
            this.appendDummyInput().appendField("CheckBallDist")
                .appendField(new Blockly.FieldDropdown([["<", "LT"], [">=", "GE"], ["==", "EQ"]]), "OP")
                .appendField(new Blockly.FieldNumber(0.5), "DIST").appendField("m");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(120);
        }
    };
    Blockly.Blocks['cond_is_winning'] = {
        init: function () {
            this.appendDummyInput().appendField("IsWinning");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(120);
        }
    };

    // ---- 3. Robot Actions ----
    Blockly.Blocks['action_kick'] = {
        init: function () {
            this.appendDummyInput().appendField("Kick").appendField("Speed:")
                .appendField(new Blockly.FieldNumber(1.3), "KICK_SPEED");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(20);
        }
    };
    Blockly.Blocks['action_chase'] = {
        init: function () {
            this.appendDummyInput().appendField("Chase").appendField("SafeDist:")
                .appendField(new Blockly.FieldNumber(0.5), "SAFE_DIST");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(20);
        }
    };
    Blockly.Blocks['action_adjust'] = {
        init: function () {
            this.appendDummyInput().appendField("Adjust").appendField("Range:")
                .appendField(new Blockly.FieldNumber(0.4), "RANGE");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(20);
        }
    };
    Blockly.Blocks['action_dribble_to_goal'] = {
        init: function () {
            this.appendDummyInput().appendField("Dribble").appendField("Dist:")
                .appendField(new Blockly.FieldNumber(2.0), "DIST");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(20);
        }
    };
    Blockly.Blocks['action_set_velocity'] = {
        init: function () {
            this.appendDummyInput().appendField("SetVelocity x:")
                .appendField(new Blockly.FieldNumber(0), "X").appendField("y:")
                .appendField(new Blockly.FieldNumber(0), "Y").appendField("θ:")
                .appendField(new Blockly.FieldNumber(0), "THETA");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(20);
        }
    };
    Blockly.Blocks['action_wait'] = {
        init: function () {
            this.appendDummyInput().appendField("Wait").appendField(new Blockly.FieldNumber(1.0), "SEC").appendField("s");
            this.setPreviousStatement(true, null); this.setNextStatement(true, null);
            this.setColour(20);
        }
    }

    // ---- 4. Strategy Actions / Subtrees ----
    const genericNodes = [
        'action_self_locate', 'action_search_ball', 'action_pass_receive', 'action_off_the_ball', 'action_go_back_in_field', 'action_calc_kick_dir',
        'subtree_cam_find_and_track', 'subtree_locate', 'subtree_find_ball', 'subtree_strategy_tree'
    ];
    genericNodes.forEach((name: string) => {
        Blockly.Blocks[name] = {
            init: function () {
                this.appendDummyInput().appendField(name.replace('action_', '').replace('subtree_', '[SUB] '));
                this.setPreviousStatement(true, null); this.setNextStatement(true, null);
                this.setColour(name.includes('subtree_') ? 160 : 290);
            }
        };
    });
}

// ===========================================
// XML Generator Logic
// ===========================================
function generateBTXml(workspace: Blockly.WorkspaceSvg): string {
    const topBlocks = workspace.getTopBlocks(true);
    if (topBlocks.length === 0) return "";
    return blockToXml(topBlocks[0], 2);
}

function getIndent(level: number) { return ' '.repeat(level * 2); }

function blockToXml(block: Blockly.Block | null, indentLevel: number): string {
    if (!block) return "";
    let xml = "";
    const type = block.type;
    const ind = getIndent(indentLevel);

    // Control Flow
    if (['bt_sequence', 'bt_reactive_sequence', 'bt_fallback', 'bt_reactive_fallback', 'bt_while_do_else'].includes(type)) {
        const tagName = type.replace('bt_', '').replace(/_(.)/g, (_m, p1) => p1.toUpperCase());
        const tag = tagName.charAt(0).toUpperCase() + tagName.slice(1);
        xml += `${ind}<${tag}>\n`;
        xml += getChildrenXml(block, indentLevel + 1);
        xml += `${ind}</${tag}>\n`;
    } else if (type === 'bt_repeat') {
        const num = block.getFieldValue('NUM');
        xml += `${ind}<Repeat num_cycles="${num}">\n`;
        xml += getChildrenXml(block, indentLevel + 1);
        xml += `${ind}</Repeat>\n`;
    }
    // Condition / Variables
    else if (type === 'cond_script_condition') {
        const code = block.getFieldValue('CODE').replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
        xml += `${ind}<ScriptCondition code="${code}" />\n`;
    } else if (type === 'action_script_execution') {
        const code = block.getFieldValue('CODE').replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
        xml += `${ind}<Script code="${code}" />\n`;
    } else if (type === 'cond_check_blackboard') {
        const key = block.getFieldValue('KEY');
        const val = block.getFieldValue('VAL');
        xml += `${ind}<CheckBlackboard value_A="{${key}}" value_B="${val}" return_on_mismatch="FAILURE" />\n`;
    } else if (type === 'cond_ball_dist') {
        xml += `${ind}<Condition ID="CheckBallDist" op="${block.getFieldValue('OP')}" dist="${block.getFieldValue('DIST')}" />\n`;
    } else if (type === 'cond_is_winning') {
        xml += `${ind}<IsWinning/>\n`;
    }
    // Robot Actions
    else if (type === 'action_kick') xml += `${ind}<Action ID="Kick" speed_limit="1.3" vx_limit="1.3" vy_limit="0.5" kick_type="kick" kick_speed="${block.getFieldValue('KICK_SPEED')}" min_msec_kick="500" />\n`;
    else if (type === 'action_chase') xml += `${ind}<Action ID="Chase" vx_limit="0.9" vy_limit="0.2" vtheta_limit="1.0" dist="0.4" safe_dist="${block.getFieldValue('SAFE_DIST')}" />\n`;
    else if (type === 'action_adjust') xml += `${ind}<Adjust range="${block.getFieldValue('RANGE')}" no_turn_threshold="0.1" vx_limit="0.5" vy_limit="0.5" vtheta_limit="1.0" />\n`;
    else if (type === 'action_dribble_to_goal') xml += `${ind}<DribbleToGoal dist_to_goal="${block.getFieldValue('DIST')}" min_speed="0.4" max_speed="0.6" />\n`;
    else if (type === 'action_set_velocity') xml += `${ind}<SetVelocity x="${block.getFieldValue('X')}" y="${block.getFieldValue('Y')}" theta="${block.getFieldValue('THETA')}"/>\n`;
    else if (type === 'action_wait') xml += `${ind}<Delay delay_msec="${block.getFieldValue('SEC') * 1000}">\n${ind}  <AlwaysSuccess/>\n${ind}</Delay>\n`;
    // Strategy Actions
    else if (type === 'action_self_locate') xml += `${ind}<SelfLocate mode="trust_direction" />\n`;
    else if (type === 'action_search_ball') xml += `${ind}<SearchBall/>\n`;
    else if (type === 'action_pass_receive') xml += `${ind}<PassReceive/>\n`;
    else if (type === 'action_off_the_ball') xml += `${ind}<OfftheballPosition dist_from_goal="2.0" />\n`;
    else if (type === 'action_go_back_in_field') xml += `${ind}<GoBackInField/>\n`;
    else if (type === 'action_calc_kick_dir') xml += `${ind}<CalcKickDir/>\n`;
    // SubTrees
    else if (type === 'subtree_cam_find_and_track') xml += `${ind}<SubTree ID="CamFindAndTrackBall" _autoremap="true" />\n`;
    else if (type === 'subtree_locate') xml += `${ind}<SubTree ID="Locate" _autoremap="true" />\n`;
    else if (type === 'subtree_find_ball') xml += `${ind}<SubTree ID="FindBall" _autoremap="true" />\n`;
    else if (type === 'subtree_strategy_tree') xml += `${ind}<SubTree ID="StrategyTree" _autoremap="true" />\n`;

    return xml;
}

function getChildrenXml(parentBlock: Blockly.Block, indentLevel: number): string {
    let xml = "";
    let childBlock = parentBlock.getInputTargetBlock("CHILDREN");
    while (childBlock) {
        xml += blockToXml(childBlock, indentLevel);
        childBlock = childBlock.getNextBlock();
    }
    return xml;
}
