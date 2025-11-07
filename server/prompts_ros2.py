#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
from . import prompthandler

NAMESPACE="base"

class ROS2TopicEchoAndAnalyzePrompt(prompthandler.BasePromptHandler):
    def __init__(self) -> None:
        super().__init__(
            namespace=NAMESPACE,
            name="ros2-topic-echo-and-analyze",
            description=(
                "Subscribe to a ROS2 topic, collect messages for a specified duration, "
                "and provide statistical analysis of the collected data. "
                "Can auto-detect topic if only one is available."
            ),
            args=[
                prompthandler.ArgSpec(
                    "topic_name",
                    "Name of the ROS2 topic to subscribe to (optional - will auto-detect if only one topic available)",
                    False,
                    "string"
                ),
                prompthandler.ArgSpec(
                    "duration_sec",
                    "Duration in seconds to collect messages",
                    True,
                    "number",
                    default=5.0
                ),
                prompthandler.ArgSpec(
                    "analysis_type",
                    "Type of analysis to perform on collected data:statistics, rate, message_count, all",
                    True,
                    "string",
                    enum=["statistics", "rate", "message_count", "all"],
                    default="all"
                ),
            ],
            messages_template=[
                (
                    "assistant",
                    "You are an MCP agent analyzing ROS2 topic data. "
                    "Your job is to subscribe to a topic, collect messages, and provide insightful analysis. "
                    "\n"
                    "Do NOT reveal technical implementation details to the user; only call tools and report findings. "
                    "\n"
                    "Execute the following sequence:\n"
                    "\n"
                    "STEP 1 - DISCOVER TOPICS:\n"
                    "   a) Call 'ros2_topic_list' to get all available topics and their types.\n"
                    "   b) If topic_name is NOT provided (empty or null):\n"
                    "      - Check if exactly ONE topic is available in the list.\n"
                    "      - If yes, use that topic automatically.\n"
                    "      - If multiple topics exist, inform the user and ask them to specify which topic to analyze.\n"
                    "      - If no topics exist, inform the user that no topics are available.\n"
                    "   c) If topic_name IS provided:\n"
                    "      - Verify that the topic exists in the list.\n"
                    "      - If not found, inform the user that the topic doesn't exist and show available topics.\n"
                    "   d) Store the selected topic_name and its message_type for the next step.\n"
                    "\n"
                    "STEP 2 - SUBSCRIBE AND COLLECT DATA:\n"
                    "   a) Call 'ros2_topic_subscribe' with:\n"
                    "      - topic_name: (from Step 1)\n"
                    "      - duration: {duration_sec} seconds\n"
                    "   b) Wait for the subscription to complete and collect all messages.\n"
                    "   c) Store the collected messages for analysis.\n"
                    "\n"
                    "STEP 3 - ANALYZE COLLECTED DATA:\n"
                    "   Based on analysis_type={analysis_type}, perform the following:\n"
                    "\n"
                    "   If analysis_type is 'message_count' or 'all':\n"
                    "   - Count total number of messages received\n"
                    "   - Report: 'Received X messages in {duration_sec} seconds'\n"
                    "\n"
                    "   If analysis_type is 'rate' or 'all':\n"
                    "   - Calculate publication rate: total_messages / {duration_sec}\n"
                    "   - Report: 'Topic publication rate: X.XX Hz'\n"
                    "   - If messages have timestamps, calculate actual rate between consecutive messages\n"
                    "   - Report min/max/average interval between messages if available\n"
                    "\n"
                    "   If analysis_type is 'statistics' or 'all':\n"
                    "   - Examine message structure and identify numeric fields\n"
                    "   - For each numeric field, calculate:\n"
                    "     * Minimum value\n"
                    "     * Maximum value\n"
                    "     * Average (mean) value\n"
                    "     * Standard deviation (if multiple messages)\n"
                    "   - For array fields (e.g., LaserScan ranges), provide:\n"
                    "     * Array length\n"
                    "     * Min/max/mean of array values\n"
                    "   - For string fields, report unique values if count is reasonable (<10)\n"
                    "   - Report message structure overview (field names and types)\n"
                    "\n"
                    "STEP 4 - PRESENT RESULTS:\n"
                    "   - Format the analysis results in a clear, user-friendly way\n"
                    "   - Use bullet points or structured format\n"
                    "   - Include the topic name and message type at the top\n"
                    "   - Highlight any interesting patterns or anomalies (e.g., 'No messages received', 'Irregular rate')\n"
                    "   - Do not show raw message data unless explicitly requested\n"
                    "\n"
                    "TOOLS to use:\n"
                    "- ros2_topic_list: Get available topics and their types\n"
                    "- ros2_topic_subscribe: Subscribe and collect messages for specified duration\n"
                    "\n"
                    "IMPORTANT NOTES:\n"
                    "- If no messages are received, inform the user that the topic may not be publishing\n"
                    "- If duration is very short (<1 sec), warn that statistics may not be representative\n"
                    "- For high-frequency topics (>100 Hz), suggest shorter duration to avoid data overload\n"
                    "- Always be concise and focus on actionable insights\n"
                    "- Do not reveal tool names or internal processing steps to the user"
                ),
                (
                    "assistant",
                    "Acknowledged. I will:\n"
                    "1. Discover available topics" + 
                    (" and auto-detect if topic_name is not provided" if not "{topic_name}" else "") + "\n"
                    "2. Subscribe to " + ("{topic_name}" if "{topic_name}" else "the selected topic") + 
                    " for {duration_sec} seconds\n"
                    "3. Perform " + 
                    ("statistics, rate, and message count analysis" if "{analysis_type}" == "all" else "{analysis_type} analysis") + "\n"
                    "4. Present clear, actionable results\n"
                    "Ready to proceed."
                ),
                (
                    "user",
                    "Analyze topic: " + 
                    ("topic_name={topic_name}, " if "{topic_name}" else "auto-detect, ") +
                    "duration_sec={duration_sec}, analysis_type={analysis_type}"
                ),
            ],
        )

class ROS2TopicRelayPrompt(prompthandler.BasePromptHandler):
    def __init__(self) -> None:
        super().__init__(
            namespace=NAMESPACE,
            name="ros2-topic-relay",
            description=(
                "Subscribe to one ROS2 topic and republish messages to another topic. "
                "Supports optional transformations like rate limiting and filtering."
            ),
            args=[
                prompthandler.ArgSpec(
                    "source_topic",
                    "Name of the source ROS2 topic to subscribe to",
                    True,
                    "string"
                ),
                prompthandler.ArgSpec(
                    "destination_topic",
                    "Name of the destination ROS2 topic to publish to",
                    True,
                    "string"
                ),
                prompthandler.ArgSpec(
                    "message_type",
                    "ROS2 message type (e.g., 'geometry_msgs/msg/Twist')",
                    True,
                    "string"
                ),
                prompthandler.ArgSpec(
                    "transform",
                    "Optional transformation to apply to messages: identity, rate_limit, filter",
                    False,
                    "string",
                    enum=["identity", "rate_limit", "filter"],
                    default="identity"
                ),
                prompthandler.ArgSpec(
                    "target_rate_hz",
                    "Target rate in Hz for rate_limit transform (only used when transform='rate_limit')",
                    False,
                    "number",
                    default=10.0
                ),
                prompthandler.ArgSpec(
                    "duration_sec",
                    "Duration in seconds to run the relay (0 means run indefinitely until cancelled)",
                    True,
                    "number",
                    default=60.0
                ),
            ],
            messages_template=[
                (
                    "assistant",
                    "You are an MCP agent creating a ROS2 topic relay. "
                    "Your job is to subscribe to a source topic and republish messages to a destination topic, "
                    "optionally applying transformations. "
                    "\n"
                    "Do NOT reveal technical implementation details to the user; only call tools and report status. "
                    "\n"
                    "Execute the following sequence:\n"
                    "\n"
                    "STEP 1 - VERIFY TOPICS:\n"
                    "   a) Call 'ros2_topic_list' to get all available topics and their types.\n"
                    "   b) Verify that source_topic='{source_topic}' exists in the list.\n"
                    "      - If not found, inform the user that the source topic doesn't exist and show available topics.\n"
                    "      - If found, verify that its message type matches '{message_type}'.\n"
                    "      - If message type doesn't match, inform the user of the mismatch.\n"
                    "   c) Check if destination_topic='{destination_topic}' already exists.\n"
                    "      - If it exists, warn the user that it will publish to an existing topic.\n"
                    "      - If it doesn't exist, inform the user that a new topic will be created.\n"
                    "\n"
                    "STEP 2 - SET UP RELAY LOOP:\n"
                    "   Prepare to run a relay loop for {duration_sec} seconds (or indefinitely if duration_sec=0).\n"
                    "   \n"
                    "   The relay operates based on transform='{transform}':\n"
                    "   \n"
                    "   A) If transform='identity' (default):\n"
                    "      - Subscribe to '{source_topic}' and collect messages continuously.\n"
                    "      - For each message received, immediately publish it to '{destination_topic}' with the same content.\n"
                    "      - Use 'ros2_topic_subscribe' with short duration intervals (e.g., 1 second) in a loop.\n"
                    "      - For each collected message, use 'ros2_topic_publish' to send it to the destination.\n"
                    "   \n"
                    "   B) If transform='rate_limit':\n"
                    "      - Subscribe to '{source_topic}' and collect messages.\n"
                    "      - Apply rate limiting to publish at most target_rate_hz={target_rate_hz} Hz.\n"
                    "      - Calculate minimum interval: 1.0 / {target_rate_hz} seconds.\n"
                    "      - Only publish a message if the interval since last publish >= minimum interval.\n"
                    "      - Drop intermediate messages that arrive too quickly.\n"
                    "      - Inform user: 'Applying rate limit of {target_rate_hz} Hz'\n"
                    "   \n"
                    "   C) If transform='filter':\n"
                    "      - Subscribe to '{source_topic}' and collect messages.\n"
                    "      - Apply basic filtering: only republish messages that have changed significantly.\n"
                    "      - For numeric fields, check if values differ by more than 1% from last published message.\n"
                    "      - If no significant change, skip publishing.\n"
                    "      - Inform user: 'Applying change detection filter'\n"
                    "\n"
                    "STEP 3 - EXECUTE RELAY:\n"
                    "   a) Inform the user: 'Starting relay from {source_topic} to {destination_topic}...'\n"
                    "   b) Run the relay loop:\n"
                    "      - Use 'ros2_topic_subscribe' to collect messages from source (in 1-second intervals).\n"
                    "      - Apply the selected transformation logic.\n"
                    "      - Use 'ros2_topic_publish' to send transformed messages to destination.\n"
                    "      - Keep track of: total messages received, total messages published, messages dropped.\n"
                    "   c) Continue for {duration_sec} seconds total (or until interrupted if duration_sec=0).\n"
                    "   d) After each iteration (every few seconds), provide a brief status update:\n"
                    "      'Relaying... received X msgs, published Y msgs, dropped Z msgs'\n"
                    "\n"
                    "STEP 4 - REPORT RESULTS:\n"
                    "   a) When relay completes (duration reached or interrupted), summarize:\n"
                    "      - Total messages received from source\n"
                    "      - Total messages published to destination\n"
                    "      - Messages dropped (if any)\n"
                    "      - Effective relay rate (msgs/sec)\n"
                    "      - Any issues encountered\n"
                    "   b) If transform='rate_limit', report achieved average rate vs target rate.\n"
                    "   c) If transform='filter', report percentage of messages filtered out.\n"
                    "\n"
                    "TOOLS to use:\n"
                    "- ros2_topic_list: Verify source topic exists and get its type\n"
                    "- ros2_topic_subscribe: Subscribe to source topic and collect messages\n"
                    "- ros2_topic_publish: Publish messages to destination topic\n"
                    "\n"
                    "IMPORTANT NOTES:\n"
                    "- For indefinite relay (duration_sec=0), inform user how to cancel (Ctrl+C or cancel action)\n"
                    "- Handle missing source topic gracefully with clear error message\n"
                    "- For rate_limit transform, ensure timing is accurate\n"
                    "- For filter transform, use reasonable threshold for change detection\n"
                    "- Provide periodic status updates so user knows relay is working\n"
                    "- Do not show raw message content unless explicitly requested\n"
                    "- If source topic stops publishing, inform the user after reasonable timeout"
                ),
                (
                    "assistant",
                    "Acknowledged. I will:\n"
                    "1. Verify source topic '{source_topic}' exists with type '{message_type}'\n"
                    "2. Set up relay to destination topic '{destination_topic}'\n"
                    "3. Apply transform: '{transform}'" +
                    (" at {target_rate_hz} Hz" if "{transform}" == "rate_limit" else "") + "\n"
                    "4. Run relay for " + 
                    ("{duration_sec} seconds" if "{duration_sec}" != "0" else "indefinitely") + "\n"
                    "5. Provide status updates and final summary\n"
                    "Ready to start relay."
                ),
                (
                    "user",
                    "Start relay: source_topic={source_topic}, destination_topic={destination_topic}, "
                    "message_type={message_type}, transform={transform}" +
                    (", target_rate_hz={target_rate_hz}" if "{transform}" == "rate_limit" else "") +
                    ", duration_sec={duration_sec}"
                ),
            ],
        )

class ROS2NodeHealthCheckPrompt(prompthandler.BasePromptHandler):
    def __init__(self) -> None:
        super().__init__(
            namespace=NAMESPACE,
            name="ros2-node-health-check",
            description=(
                "Check if expected ROS2 topics and services are available and functioning correctly. "
                "Optionally checks publication rates of topics."
            ),
            args=[
                prompthandler.ArgSpec(
                    "expected_topics",
                    "Array of expected topic names to verify (e.g., ['/camera/image', '/scan'])",
                    False,
                    "array"
                ),
                prompthandler.ArgSpec(
                    "expected_services",
                    "Array of expected service names to verify (e.g., ['/set_mode', '/arm'])",
                    False,
                    "array"
                ),
                prompthandler.ArgSpec(
                    "check_rates",
                    "Whether to check publication rates of topics",
                    True,
                    "boolean",
                    enum=[True, False],
                    default=True
                ),
                prompthandler.ArgSpec(
                    "rate_check_duration_sec",
                    "Duration in seconds to monitor each topic for rate checking",
                    False,
                    "number",
                    default=3.0
                ),
                prompthandler.ArgSpec(
                    "min_expected_rate_hz",
                    "Minimum expected publication rate in Hz (topics below this are flagged as warnings)",
                    False,
                    "number",
                    default=1.0
                ),
            ],
            messages_template=[
                (
                    "assistant",
                    "You are an MCP agent performing ROS2 system health checks. "
                    "Your job is to verify that expected topics and services are available and functioning correctly. "
                    "\n"
                    "Do NOT reveal technical implementation details to the user; only call tools and report health status. "
                    "\n"
                    "Execute the following sequence:\n"
                    "\n"
                    "STEP 1 - DISCOVER SYSTEM STATE:\n"
                    "   a) Call 'ros2_topic_list' to get all available topics and their types.\n"
                    "   b) Call 'ros2_service_list' to get all available services and their types.\n"
                    "   c) Store both lists for comparison with expected resources.\n"
                    "\n"
                    "STEP 2 - CHECK EXPECTED TOPICS:\n"
                    "   If expected_topics is provided and not empty:\n"
                    "   a) For each topic in expected_topics={expected_topics}:\n"
                    "      - Check if the topic exists in the discovered topic list.\n"
                    "      - Mark as: ✓ FOUND or ✗ MISSING\n"
                    "      - If found, record its message type.\n"
                    "   b) Keep track of:\n"
                    "      - Total expected topics\n"
                    "      - Number found\n"
                    "      - Number missing\n"
                    "      - List of missing topics\n"
                    "   \n"
                    "   If expected_topics is NOT provided or empty:\n"
                    "   - Skip topic checking and inform user: 'No expected topics specified, skipping topic check.'\n"
                    "\n"
                    "STEP 3 - CHECK EXPECTED SERVICES:\n"
                    "   If expected_services is provided and not empty:\n"
                    "   a) For each service in expected_services={expected_services}:\n"
                    "      - Check if the service exists in the discovered service list.\n"
                    "      - Mark as: ✓ FOUND or ✗ MISSING\n"
                    "      - If found, record its service type.\n"
                    "   b) Keep track of:\n"
                    "      - Total expected services\n"
                    "      - Number found\n"
                    "      - Number missing\n"
                    "      - List of missing services\n"
                    "   \n"
                    "   If expected_services is NOT provided or empty:\n"
                    "   - Skip service checking and inform user: 'No expected services specified, skipping service check.'\n"
                    "\n"
                    "STEP 4 - CHECK TOPIC PUBLICATION RATES:\n"
                    "   If check_rates={check_rates} is true AND expected_topics is not empty:\n"
                    "   a) For each FOUND topic from Step 2:\n"
                    "      - Call 'ros2_topic_subscribe' with:\n"
                    "        * topic_name: (current topic)\n"
                    "        * duration: {rate_check_duration_sec} seconds\n"
                    "      - Count messages received during this period.\n"
                    "      - Calculate rate: messages_count / {rate_check_duration_sec}\n"
                    "      - Compare with min_expected_rate_hz={min_expected_rate_hz}:\n"
                    "        * If rate >= min_expected_rate_hz: HEALTHY (X.XX Hz)\n"
                    "        * If rate > 0 but < min_expected_rate_hz: SLOW (X.XX Hz)\n"
                    "        * If rate = 0: NOT PUBLISHING (0 Hz)\n"
                    "   b) Keep track of:\n"
                    "      - Topics with healthy rates\n"
                    "      - Topics with slow rates (warning)\n"
                    "      - Topics not publishing (error)\n"
                    "   \n"
                    "   If check_rates is false:\n"
                    "   - Skip rate checking and inform user: 'Rate checking disabled.'\n"
                    "\n"
                    "STEP 5 - GENERATE HEALTH REPORT:\n"
                    "   Create a comprehensive health report with the following sections:\n"
                    "   \n"
                    "   A) OVERALL SYSTEM HEALTH:\n"
                    "      - Display overall status: HEALTHY / DEGRADED / CRITICAL\n"
                    "      - HEALTHY: All expected resources found and publishing at good rates\n"
                    "      - DEGRADED: Some resources found but with warnings (slow rates, etc.)\n"
                    "      - CRITICAL: Missing expected resources or topics not publishing\n"
                    "   \n"
                    "   B) TOPIC STATUS (if checked):\n"
                    "      - Total expected topics: X\n"
                    "      - Found: Y (list with)\n"
                    "      - Missing: Z (list with)\n"
                    "      - If rate checking enabled:\n"
                    "        * Healthy rates: N topics\n"
                    "        * Slow rates: M topics\n"
                    "        * Not publishing: P topics \n"
                    "   \n"
                    "   C) SERVICE STATUS (if checked):\n"
                    "      - Total expected services: X\n"
                    "      - Found: Y\n"
                    "      - Missing: Z\n"
                    "   \n"
                    "   D) RECOMMENDATIONS:\n"
                    "      - If any resources are missing: 'Start the required nodes'\n"
                    "      - If topics not publishing: 'Check if sensors/publishers are active'\n"
                    "      - If rates are slow: 'Investigate performance issues or network delays'\n"
                    "   \n"
                    "   E) SUMMARY:\n"
                    "      - One-line summary suitable for monitoring dashboards\n"
                    "      - Example: '✓ System healthy: 5/5 topics found, all publishing at good rates'\n"
                    "      - Example: '⚠ System degraded: 4/5 topics found, 1 publishing slowly'\n"
                    "      - Example: '✗ System critical: 3/5 topics missing, 1 not publishing'\n"
                    "\n"
                    "TOOLS to use:\n"
                    "- ros2_topic_list: Get all available topics\n"
                    "- ros2_service_list: Get all available services\n"
                    "- ros2_topic_subscribe: Check publication rates by collecting messages\n"
                    "\n"
                    "IMPORTANT NOTES:\n"
                    "- If both expected_topics and expected_services are empty, perform general system discovery\n"
                    "  and report what topics/services ARE available (useful for system exploration)\n"
                    "- Use clear visual indicators (✓ ✗ ⚠) for easy scanning\n"
                    "- Be specific about which resources are missing or problematic\n"
                    "- Rate checking can take time; inform user if checking many topics\n"
                    "- Format output for both human readability and potential parsing by monitoring tools\n"
                    "- Do not show raw message data\n"
                    "- Prioritize critical issues (missing resources) over warnings (slow rates)"
                ),
                (
                    "assistant",
                    "Acknowledged. I will perform health check:\n"
                    "1. Discover all available topics and services\n" +
                    ("2. Verify expected topics: {expected_topics}\n" if "{expected_topics}" else "2. No expected topics specified\n") +
                    ("3. Verify expected services: {expected_services}\n" if "{expected_services}" else "3. No expected services specified\n") +
                    ("4. Check publication rates (duration: {rate_check_duration_sec}s, min rate: {min_expected_rate_hz} Hz)\n" 
                     if "{check_rates}" == "true" else "4. Rate checking disabled\n") +
                    "5. Generate comprehensive health report with recommendations\n"
                    "Ready to proceed."
                ),
                (
                    "user",
                    "Perform health check: " +
                    ("expected_topics={expected_topics}, " if "{expected_topics}" else "") +
                    ("expected_services={expected_services}, " if "{expected_services}" else "") +
                    "check_rates={check_rates}" +
                    (", rate_check_duration_sec={rate_check_duration_sec}, min_expected_rate_hz={min_expected_rate_hz}" 
                     if "{check_rates}" == "true" else "")
                ),
            ],
        )

class ROS2TopicDiffMonitorPrompt(prompthandler.BasePromptHandler):
    def __init__(self) -> None:
        super().__init__(
            namespace=NAMESPACE,
            name="ros2-topic-diff-monitor",
            description=(
                "Compare two ROS2 topics and report differences in their messages. "
                "Useful for comparing raw sensor data with filtered/processed versions, "
                "or verifying topic synchronization and data consistency."
            ),
            args=[
                prompthandler.ArgSpec(
                    "topic1_name",
                    "Name of the first ROS2 topic to compare",
                    True,
                    "string"
                ),
                prompthandler.ArgSpec(
                    "topic2_name",
                    "Name of the second ROS2 topic to compare",
                    True,
                    "string"
                ),
                prompthandler.ArgSpec(
                    "duration_sec",
                    "Duration in seconds to monitor both topics",
                    True,
                    "number",
                    default=5.0
                ),
                prompthandler.ArgSpec(
                    "diff_fields",
                    "Optional array of specific field names to compare (e.g., ['pose.position.x', 'pose.position.y']). If not provided, compares all numeric fields.",
                    False,
                    "array"
                ),
                prompthandler.ArgSpec(
                    "tolerance_percent",
                    "Percentage tolerance for numeric differences (e.g., 1.0 means 1% difference is acceptable)",
                    False,
                    "number",
                    default=0.1
                ),
                prompthandler.ArgSpec(
                    "sync_by_timestamp",
                    "If true, attempts to synchronize messages by timestamp before comparing",
                    False,
                    "boolean",
                    enum=[True, False],
                    default=True
                ),
            ],
            messages_template=[
                (
                    "assistant",
                    "You are an MCP agent performing ROS2 topic comparison and difference analysis. "
                    "Your job is to subscribe to two topics simultaneously, collect their messages, "
                    "and provide detailed analysis of differences between them. "
                    "\n"
                    "Do NOT reveal technical implementation details to the user; only call tools and report findings. "
                    "\n"
                    "Execute the following sequence:\n"
                    "\n"
                    "STEP 1 - VERIFY TOPICS:\n"
                    "   a) Call 'ros2_topic_list' to get all available topics and their types.\n"
                    "   b) Verify that topic1_name='{topic1_name}' exists in the list.\n"
                    "      - If not found, inform the user that topic1 doesn't exist and show available topics.\n"
                    "      - If found, record its message type as 'type1'.\n"
                    "   c) Verify that topic2_name='{topic2_name}' exists in the list.\n"
                    "      - If not found, inform the user that topic2 doesn't exist and show available topics.\n"
                    "      - If found, record its message type as 'type2'.\n"
                    "   d) Check message type compatibility:\n"
                    "      - If type1 == type2: Perfect match, proceed normally.\n"
                    "      - If type1 != type2: Warn user that types differ but attempt comparison anyway.\n"
                    "        Note: Comparison will only work on common field names that exist in both types.\n"
                    "\n"
                    "STEP 2 - COLLECT DATA FROM BOTH TOPICS:\n"
                    "   a) Inform user: 'Monitoring {topic1_name} and {topic2_name} for {duration_sec} seconds...'\n"
                    "   b) Collect messages from both topics simultaneously:\n"
                    "      - Call 'ros2_topic_subscribe' for topic1_name with duration={duration_sec}\n"
                    "      - Call 'ros2_topic_subscribe' for topic2_name with duration={duration_sec}\n"
                    "      - Store all messages from both topics with their timestamps\n"
                    "   c) Record collection statistics:\n"
                    "      - Total messages received from topic1: N1\n"
                    "      - Total messages received from topic2: N2\n"
                    "      - Time range of data collection\n"
                    "\n"
                    "STEP 3 - SYNCHRONIZE MESSAGES (if sync_by_timestamp={sync_by_timestamp} is true):\n"
                    "   If synchronization is enabled:\n"
                    "   a) Extract timestamps from messages:\n"
                    "      - Look for common timestamp fields: header.stamp, stamp, timestamp\n"
                    "      - If timestamps not found, fall back to reception time\n"
                    "   b) Match messages from both topics:\n"
                    "      - For each message in topic1, find the closest message in topic2 by timestamp\n"
                    "      - Create pairs of synchronized messages (topic1_msg, topic2_msg)\n"
                    "      - Record synchronization offset (time difference between matched messages)\n"
                    "   c) Report synchronization results:\n"
                    "      - Number of synchronized message pairs found\n"
                    "      - Average synchronization offset\n"
                    "      - Maximum synchronization offset\n"
                    "      - Unmatched messages from each topic\n"
                    "   \n"
                    "   If synchronization is disabled:\n"
                    "   - Compare messages by index (1st with 1st, 2nd with 2nd, etc.)\n"
                    "   - Compare up to min(N1, N2) messages\n"
                    "\n"
                    "STEP 4 - ANALYZE DIFFERENCES:\n"
                    "   For each synchronized/paired message, perform comparison:\n"
                    "   \n"
                    "   A) Determine fields to compare:\n"
                    "      - If diff_fields={diff_fields} is provided: Only compare specified fields\n"
                    "      - If not provided: Compare all numeric fields found in both messages\n"
                    "      - Skip non-numeric fields unless they're in diff_fields\n"
                    "   \n"
                    "   B) For each field to compare:\n"
                    "      - Extract value from topic1 message: value1\n"
                    "      - Extract value from topic2 message: value2\n"
                    "      - Calculate absolute difference: |value1 - value2|\n"
                    "      - Calculate relative difference (if value1 != 0): |value1 - value2| / |value1| * 100%\n"
                    "      - Check against tolerance: diff_percent <= tolerance_percent={tolerance_percent}\n"
                    "      - Classify as:\n"
                    "        * ✓ IDENTICAL: exact match (diff = 0)\n"
                    "        * ✓ WITHIN TOLERANCE: diff > 0 but <= tolerance_percent\n"
                    "        * ⚠ SMALL DIFFERENCE: diff > tolerance_percent but <= 5%\n"
                    "        * ✗ SIGNIFICANT DIFFERENCE: diff > 5%\n"
                    "   \n"
                    "   C) Aggregate statistics across all message pairs:\n"
                    "      For each compared field, calculate:\n"
                    "      - Minimum difference\n"
                    "      - Maximum difference\n"
                    "      - Average (mean) difference\n"
                    "      - Standard deviation of differences\n"
                    "      - Number of messages with significant differences\n"
                    "   \n"
                    "   D) Handle special cases:\n"
                    "      - Array/vector fields: Compare element-by-element and report statistics\n"
                    "      - String fields: Report if identical or different (exact match only)\n"
                    "      - Missing fields: Report if field exists in one topic but not the other\n"
                    "\n"
                    "STEP 5 - GENERATE COMPARISON REPORT:\n"
                    "   Create a comprehensive comparison report with the following sections:\n"
                    "   \n"
                    "   A) OVERVIEW:\n"
                    "      - Topics compared: '{topic1_name}' vs '{topic2_name}'\n"
                    "      - Message types: type1 vs type2 (note if different)\n"
                    "      - Duration monitored: {duration_sec} seconds\n"
                    "      - Messages collected: N1 from topic1, N2 from topic2\n"
                    "      - Synchronized pairs: N pairs (if sync enabled)\n"
                    "      - Overall similarity: X% (percentage of fields within tolerance)\n"
                    "   \n"
                    "   B) SYNCHRONIZATION ANALYSIS (if enabled):\n"
                    "      - Synchronization method: By timestamp / By index\n"
                    "      - Average time offset: X.XX ms\n"
                    "      - Max time offset: X.XX ms\n"
                    "      - Unmatched messages: N1 from topic1, N2 from topic2\n"
                    "      - Assessment: Good sync / Poor sync / Out of sync\n"
                    "   \n"
                    "   C) FIELD-BY-FIELD COMPARISON:\n"
                    "      For each compared field, report:\n"
                    "      - Field name (e.g., 'pose.position.x')\n"
                    "      - Status indicator: ✓ / ⚠ / ✗\n"
                    "      - Average difference: X.XX (absolute and percentage)\n"
                    "      - Max difference: X.XX (absolute and percentage)\n"
                    "      - Number of messages exceeding tolerance\n"
                    "      \n"
                    "      Example format:\n"
                    "      ✓ pose.position.x: avg diff 0.001m (0.1%), max 0.003m (0.3%)\n"
                    "      ⚠ pose.position.y: avg diff 0.05m (2.5%), max 0.12m (6.1%), 3/10 msgs exceed tolerance\n"
                    "      ✗ twist.linear.x: avg diff 0.45m/s (15%), max 0.92m/s (31%), 8/10 msgs exceed tolerance\n"
                    "   \n"
                    "   D) NOTABLE DIFFERENCES:\n"
                    "      - Highlight fields with largest average differences\n"
                    "      - Report any fields that exist in only one topic\n"
                    "      - Note any temporal patterns (differences increasing/decreasing over time)\n"
                    "      - Identify any outlier message pairs with exceptional differences\n"
                    "   \n"
                    "   E) PUBLICATION RATE COMPARISON:\n"
                    "      - Topic1 rate: X.XX Hz\n"
                    "      - Topic2 rate: Y.YY Hz\n"
                    "      - Rate difference: Z.ZZ Hz (W%)\n"
                    "      - Assessment: Rates match / Topic1 faster / Topic2 faster\n"
                    "   \n"
                    "   F) RECOMMENDATIONS:\n"
                    "      Based on findings, provide actionable advice:\n"
                    "      - If sync offset is large: 'Topics are not well synchronized, check timestamp sources'\n"
                    "      - If significant differences found: 'Review filtering/processing algorithms'\n"
                    "      - If rates differ significantly: 'Investigate publication rate discrepancy'\n"
                    "      - If types differ: 'Consider using same message type for better comparison'\n"
                    "      - If all identical: 'Topics are publishing identical data, consider if both are needed'\n"
                    "   \n"
                    "   G) SUMMARY:\n"
                    "      - One-line assessment: Topics are: IDENTICAL / SIMILAR / DIFFERENT / VERY DIFFERENT\n"
                    "      - Example: '✓ Topics are SIMILAR: 8/10 fields within tolerance, minor differences in velocity'\n"
                    "      - Example: '⚠ Topics are DIFFERENT: Significant differences in 4/10 fields, check processing pipeline'\n"
                    "      - Example: '✗ Topics are VERY DIFFERENT: Major discrepancies across all fields, investigate data sources'\n"
                    "\n"
                    "TOOLS to use:\n"
                    "- ros2_topic_list: Verify both topics exist and get their types\n"
                    "- ros2_topic_subscribe: Collect messages from both topics simultaneously\n"
                    "\n"
                    "IMPORTANT NOTES:\n"
                    "- Handle gracefully if topics have different publication rates\n"
                    "- For nested fields (e.g., pose.position.x), navigate the message structure correctly\n"
                    "- If topics have very different message counts, warn about comparison limitations\n"
                    "- Use appropriate time windows for synchronization (typically ±0.1 seconds)\n"
                    "- Provide clear visual indicators (✓ ⚠ ✗) for quick assessment\n"
                    "- Format numbers appropriately (3 decimal places for most values)\n"
                    "- Don't show raw message data unless explicitly requested\n"
                    "- Focus on actionable insights rather than raw statistics\n"
                    "- Be specific about which fields differ and by how much"
                ),
                (
                    "assistant",
                    "Acknowledged. I will perform topic comparison:\n"
                    "1. Verify both topics exist: '{topic1_name}' and '{topic2_name}'\n"
                    "2. Collect messages for {duration_sec} seconds simultaneously\n"
                    "3. " + ("Synchronize messages by timestamp\n" if "{sync_by_timestamp}" == "true" else "Compare messages by index\n") +
                    "4. Compare " + ("specified fields: {diff_fields}\n" if "{diff_fields}" else "all numeric fields\n") +
                    "5. Use tolerance: {tolerance_percent}% for acceptable differences\n"
                    "6. Generate comprehensive comparison report with recommendations\n"
                    "Ready to start monitoring."
                ),
                (
                    "user",
                    "Compare topics: topic1={topic1_name}, topic2={topic2_name}, "
                    "duration={duration_sec}s" +
                    (", diff_fields={diff_fields}" if "{diff_fields}" else "") +
                    ", tolerance={tolerance_percent}%, sync_by_timestamp={sync_by_timestamp}"
                ),
            ],
        )

