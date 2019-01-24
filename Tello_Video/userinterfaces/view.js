// Called 'view', this is actually the controller in MVC
//
class Commander {
    constructor(port) {

        this.socket = new WebSocket('ws://' + location.hostname + ':' + (port || 8000));
        this.socketReady = false;

        this.socket.addEventListener('open', (event) => {
            this.socketReady = true;
        });
        this.socket.onclose = () => {
            this.socket = null;
            this.socketReady = false;
        }
        this.socket.onerror = () => {
            this.socket = null;
            this.socketReady = false;
        }
    }

    sendCommand(type, payload) {
        if(this.socketReady) {
            var message ={
                'type': type,
                'payload': payload
            };
            this.socket.send(JSON.stringify(message));
            console.log('Sending', message);
        }
    }

    setPosition(x, y, z, yawx, yawy, yawz) {
        this.sendCommand('set_vp', {x: x, y: y, z: z, yaw_x: yawx, yaw_y: yawy, yaw_z: yawz});
    }

    // Instruct the flying cam to take the current local viewpoint
    setSnapshot() {
        this.sendCommand('local_vp');
    }


    /*
    setUnityReady() {
        console.log('Set unity ready!');
        this._unityReady = true;
    }
    */

}

class UnityCommander extends Commander {
    constructor(port) {
        this._unityReady = false;
        super(port);
        this.handleMessage();
    }

    handleMessage() {
        this.socket.addEventListener('message', (event) => {
            //console.log('Message from WS', event.data);
            var message = JSON.parse(event.data);
            var type = message['type'],
                payload = message['payload'];
            if(type === 'current_vp') {
                let translation = payload['translation'];
                let rotation = payload['rotation'];
                let spatial = translation.concat(rotation);
                if(gameInstance && this._unityReady) {
                    gameInstance.SendMessage('Flying Cam', 'Move', spatial.join(','));
                }
            }
        });
    }

    setUnityReady() {
        console.log('Set unity ready!');
        this._unityReady = true;
    }
}

class PlaneCommander extends Commander {
    constructor(port, drawer, widgets) {
        super(port);
        this._drawer = drawer;
        this._widgets = widgets;
        this._drawer.setWsCallbacks({
            sendVp: this.setPosition.bind(this)
        });
        this._widgets.setHeightSliderCB(this.setHeight.bind(this));
        //this._heightSet = 1500;
        this.handleMessage();
    }

    handleMessage() {
        this.socket.addEventListener('message', (event) => {
            //console.log('Message from WS', event.data);
            var message = JSON.parse(event.data);
            var type = message['type'],
                payload = message['payload'];
            /* Current viewpoint position */
            if(type === 'current_vp') {
                /* 'translation' is a three element array */
                let translation = payload['translation'];
                /* 'rotation' is a nine element array, representing a column major rotation matrix */
                let rotation = payload['rotation'];
                this._drawer.updateViewpoint(translation, rotation);
                //this._widgets.updateHeightSlider(translation[2]);
            }
            /* Current human position */
            if(type === 'current_hm') {
                let translation = payload['translation'];
                let rotation = payload['rotation'];
                this._drawer.updateHuman(translation, rotation);
            }
        });
    }

    setHeight(h) {
        this.sendCommand('set_vp_height', {'height': h});
    } 


}

var uiWidgets = new Widgets();
var uiDrawer = new Drawer('main-canvas', uiWidgets);
var commander = new PlaneCommander(8080, uiDrawer, uiWidgets);

