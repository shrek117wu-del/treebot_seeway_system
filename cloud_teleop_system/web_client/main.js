// For local testing: ws://127.0.0.1:8765
// For remote (Jetson on another machine): replace with ws://[SERVER_IP]:8765
const SIGNALING_SERVER_BASE_URL = "ws://127.0.0.1:8765";
const ROOM_ID = "room1";
const SIGNALING_SERVER_URL = `${SIGNALING_SERVER_BASE_URL}/?room=${ROOM_ID}&role=client`;

let ws;
let pc;
let dataChannel;
let controlLoopInterval;

const remoteVideo = document.getElementById('remoteVideo');
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const sigStatus = document.getElementById('sigStatus');
const rtcStatus = document.getElementById('rtcStatus');
const dataStatus = document.getElementById('dataStatus');
const gamepadStatus = document.getElementById('gamepadStatus');

// Connect to signaling server and initialize WebRTC
function startConnection() {
    ws = new WebSocket(SIGNALING_SERVER_URL);

    ws.onopen = () => {
        setStatus(sigStatus, 'Connected', true);
        connectBtn.disabled = true;
        disconnectBtn.disabled = false;
        createPeerConnection();
    };

    ws.onmessage = async (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === 'offer') {
            await pc.setRemoteDescription(new RTCSessionDescription(msg));
            const answer = await pc.createAnswer();
            await pc.setLocalDescription(answer);
            ws.send(JSON.stringify(pc.localDescription));
        } else if (msg.type === 'answer') {
            await pc.setRemoteDescription(new RTCSessionDescription(msg));
        } else if (msg.candidate) {
            await pc.addIceCandidate(new RTCIceCandidate(msg.candidate));
        }
    };

    ws.onclose = () => {
        setStatus(sigStatus, 'Disconnected - Reconnecting...', false);
        stopConnection(false); // Don't fully reset UI
        setTimeout(startConnection, 2000); // Auto-reconnect every 2 seconds
    };
}

function createPeerConnection() {
    const config = {
        iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
    };
    pc = new RTCPeerConnection(config);

    pc.onicecandidate = (event) => {
        if (event.candidate) {
            ws.send(JSON.stringify({ candidate: event.candidate }));
        }
    };

    pc.oniceconnectionstatechange = () => {
        if (pc.iceConnectionState === 'connected' || pc.iceConnectionState === 'completed') {
            setStatus(rtcStatus, 'Connected', true);
            startGamepadLoop();
        } else if (pc.iceConnectionState === 'disconnected' || pc.iceConnectionState === 'failed') {
            setStatus(rtcStatus, 'Disconnected', false);
        }
    };

    pc.ontrack = (event) => {
        if (remoteVideo.srcObject !== event.streams[0]) {
            remoteVideo.srcObject = event.streams[0];
            console.log('Received remote video stream');
        }
    };

    // Create a data channel for sub-10ms control over UDP
    // Ordered=false, maxRetransmits=0 ensures zero head-of-line blocking
    dataChannel = pc.createDataChannel('teleop_controls', {
        ordered: false,
        maxRetransmits: 0
    });

    dataChannel.onopen = () => setStatus(dataStatus, 'Open (UDP)', true);
    dataChannel.onclose = () => setStatus(dataStatus, 'Closed', false);

    // If we're the initiator, we create an offer. In our architecture, the Jetson could be the offerer,
    // but here we let the web client initiate.
    pc.createOffer().then(offer => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        ws.send(JSON.stringify(pc.localDescription));
    }).catch(console.error);
}
// ---- Gamepad Polling & Telemetry ---- //
function startGamepadLoop() {
    controlLoopInterval = setInterval(() => {
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        if (!gamepads || !gamepads[0]) {
            setStatus(gamepadStatus, 'Not Found', false);
            return;
        }

        const gp = gamepads[0];
        setStatus(gamepadStatus, `Active (${gp.id.substring(0, 15)}...)`, true);

        const linear_x = -gp.axes[1]; 
        const angular_z = -gp.axes[2];

        const command = {
            linear: { x: Math.abs(linear_x) > 0.05 ? linear_x : 0.0 },
            angular: { z: Math.abs(angular_z) > 0.05 ? angular_z : 0.0 },
            buttons: gp.buttons.map(b => b.pressed),
            timestamp: Date.now()
        };

        if (dataChannel && dataChannel.readyState === 'open') {
            dataChannel.send(JSON.stringify(command));
        }
    }, 20); // 50Hz control update loop (20ms interval) for ultra-low latency smoothness
}

function stopConnection(userInitiated = true) {
    if (controlLoopInterval) clearInterval(controlLoopInterval);
    if (dataChannel) dataChannel.close();
    if (pc) pc.close();
    if (ws && userInitiated) {
        ws.onclose = null; // Prevent reconnect loop if user manually disconnected
        ws.close();
    }
    
    if (userInitiated) {
        connectBtn.disabled = false;
        disconnectBtn.disabled = true;
        setStatus(sigStatus, 'Disconnected', false);
    }
    
    setStatus(rtcStatus, 'Disconnected', false);
    setStatus(dataStatus, 'Closed', false);
    setStatus(gamepadStatus, 'Not Found', false);
}

function setStatus(element, text, isGood) {
    element.textContent = text;
    element.className = 'status-value ' + (isGood ? 'connected' : 'disconnected');
}

connectBtn.addEventListener('click', startConnection);
disconnectBtn.addEventListener('click', () => stopConnection(true));
