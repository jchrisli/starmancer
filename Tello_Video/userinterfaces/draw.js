// Jiannan Li@DGP, created Nov 2018

// Very simple ui for controlling a drone

/* One long and one short line visualizing location and orientation */
class OrientedFigure {
    constructor(x, y, ori, scale, style, can) {
        this._x = x;
        this._y = y;
        this._ori = ori;
        this._scale = scale;
        this._style = style;
        this._can = can;
        this._LONG = 100;
        this._SHORT = 40;
        this._lineLong = null;
        this._lineShort = null;
        this._g = null;
        this._Render();
    }
    /* Render it on the canvas according to the states*/
    _Render() {
        /* Draw it at the origin */
        let paper = this._can;
        this._lineLong = paper.line(-this._LONG / 2, this._SHORT / 2, this._LONG / 2, this._SHORT / 2);
        this._lineShort = paper.line(0, this._SHORT / 2, 0, -this._SHORT / 2);
        let defaultStyle = {
            stroke: 'black',
            strokeWidth: 2
        };
        let styleToApply = this._style ? this._style : defaultStyle;
        this._lineLong.attr(styleToApply);
        this._lineShort.attr(styleToApply);
        this._g = paper.g(this._lineLong, this._lineShort);
        this._ApplyTransform();
    }

    _ApplyTransform() {
        const scaleStr = 's' + this._scale + ',' + this._scale,
            rotStr = 'r' + this._ori,
            //rotStr = '',
            translateStr = 't' + this._x + ',' + this._y,
            //translateStr = '',
            transformStr =  translateStr + rotStr + scaleStr;
        this._g.transform(transformStr);
    }

    Update(x, y, ori) {
        /* Do all the transformations */
        /* Scale, translate, rotate */
        this._x = x;
        this._y = y;
        this._ori = ori;
        this._ApplyTransform();
    }

    get X() {
        return this._x;
    }

    get Y() {
        return this._y;
    }

    get Ori() {
        return this._ori;
    }
}

/* You cannot pull it out of a desk */
/* Manage UI drawing stuffs */
class Drawer {
    constructor(canElemId, widgets) {
        this._can = Snap('#' + canElemId);
        this._canElemId = canElemId;
        this._canElem = document.getElementById(canElemId);

        /* Set transformation constants */
        let alpha = 400 / (19.5 * 25.4 * 5);
        /* This is column major */
        /* xx2d -> transform to 2d */
        this.pt2dMat = glMatrix.mat4.fromValues(0, alpha, 0, 0, alpha, 0, 0, 0, 0, 0, 1, 0, 500, 150, 0, 1),
        this.dir2dMat = glMatrix.mat4.fromValues(0, alpha, 0, 0, alpha, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
        this.pt3dMat = glMatrix.mat4.create(),
        this.dir3dMat = glMatrix.mat4.create();
        glMatrix.mat4.invert(this.pt3dMat, this.pt2dMat);
        glMatrix.mat4.invert(this.dir3dMat, this.dir2dMat);

        /* Put the figure for the human and the viewpoint on the screen */
        this._vp = new OrientedFigure(100, 100, 0, 0.75, null, this._can);
        this._hm = new OrientedFigure(300, 100, 0, 1.25, null, this._can);
        /* Show where the set target is */
        this._tg = new OrientedFigure(500, 100, 0, 0.75, {
            stroke: 'black',
            strokeWidth: 2,
            strokeOpacity: 0.5 
        }, this._can);
        this._lookTargetSym = this._can.circle(0, 0, 10);
        this._lookTargetSym.addClass('invisible');

        this._wsCallbacks = null;
        this._isMouseDown = false;
        this._mouseDownOnCanvas = {x: -1, y: -1};
        this._mouseMoveCount = 0;
        this._can.mousedown(this._recordMouseDown.bind(this));
        this._can.mousemove(this._updateMousePos.bind(this));
        this._can.mouseup(this._setViewpoint.bind(this));
        this._widgets = widgets;
    }

    setWsCallbacks(cb) {
        this._wsCallbacks = cb;
    }

    /* Get the signed angle between two 2d vectors, in degrees, positive for clockwise */
    _signedAngle (v1, v2) {
        let cross = glMatrix.vec3.create(),
            angle = glMatrix.vec2.angle(v1, v2);
        glMatrix.vec2.cross(cross, v1, v2);

        return (cross[2] > 0 ? angle : -angle) / Math.PI * 180;
    } 

    _setViewpoint(event) {
        /* Reposition the viewpoint figure where the canvas is clicked. Reorient it towards the human*/
        //if(event.target.id === this._canElemId) {
        let moveTargetCanvas = this._screenToCanvas({x: event.x, y: event.y});
        if(this._mouseMoveCount > 1) {
            let lookTargetCanvas = this._mouseDownOnCanvas;
            this._canvasPt2Vp(moveTargetCanvas, lookTargetCanvas);
        } else {
            /* If it's a single click, set the look target to the current point of interest */
            this._canvasPt2Vp(moveTargetCanvas, {x: this._hm.X, y: this._hm.Y});
            this._lookTargetSym.addClass('invisible');
        }
        if(this._wsCallbacks && this._wsCallbacks.sendVp) {
            /* Transform to 3d space */
            let in3d = this._transform3d(this._tg.X, this._tg.Y, this._tg.Ori);
            this._wsCallbacks.sendVp(...in3d);
        }
        this._isMouseDown = false;
        this._mouseMoveCount = 0;
        //}
    }

    _screenToCanvas(screenPt) {
        let pt = this._canElem.createSVGPoint();
        pt.x = screenPt.x;
        pt.y = screenPt.y;
        return pt.matrixTransform(this._canElem.getScreenCTM().inverse());    
    }

    /* Note: all coordinates in SVG frame */
    _canvasPt2Vp(xy, lookXY) {
        /* Make the viewpoint always face the human */
        let lookDir = glMatrix.vec2.fromValues(lookXY.x - xy.x, lookXY.y - xy.y),
            baseDir = glMatrix.vec2.fromValues(0, -1),
            angle = this._signedAngle(baseDir, lookDir);
        /* FIXIT: Should it be done here? */
        this._tg.Update(xy.x, xy.y, angle);
    }

    _recordMouseDown(evt) {
        this._mouseDownOnCanvas = this._screenToCanvas({x: evt.x, y: evt.y});
        this._isMouseDown = true;
        /* Draw something for the target */
        //this._can.circle(this._lookTarget.x, this._lookTarget.y, 10);

    }

    /* For the drag gesture */
    _updateMousePos(evt) {
        if(this._isMouseDown) {
            this._mouseMoveCount = this._mouseMoveCount + 1;
            if(this._mouseMoveCount === 1) {
                /* only draw the starting point if it's a drag */
                //this._can.circle(this._mouseDownOnCanvas.x, this._mouseDownOnCanvas.y, 10);
                this._lookTargetSym.removeClass('invisible');
                this._lookTargetSym.attr({cx: this._mouseDownOnCanvas.x, cy: this._mouseDownOnCanvas.y});
            }
        }
        /* TODO: draw something? */
    }

    /* Transform from the plane view to the 3D space */
    /* Return [x, y, z, yaw_x, yaw_y, yaw_z] */
    _transform3d(x, y, angle) {
        let pos = glMatrix.vec4.fromValues(x, y, this._widgets.getSetHeight(), 1),
            dir = glMatrix.vec4.fromValues(-1, 0, 0, 1),
            rotMat = glMatrix.mat4.create();
        glMatrix.mat4.fromRotation(rotMat, angle / 180 * Math.PI, [0, 0, -1]);
        glMatrix.vec4.transformMat4(dir, dir, rotMat);
        //glMatrix.vec4.transformMat4(dir, dir, this.dir3dMat);
        glMatrix.vec4.transformMat4(pos, pos, this.pt3dMat);
        return [pos[0], pos[1], pos[2], dir[0], dir[1], dir[2]];
    }

    /* Transform viewpoint position and orientation in 3D to the 2D plane */
    /* Return [x_in_2d, y_in_2d, angle] */ 
    _transform2d (pt3d, rot3d, forward3d) {
        let pt3dv4 = glMatrix.vec4.fromValues(pt3d[0], pt3d[1], pt3d[2], 1),
            pt2dv4 = glMatrix.vec4.create();
        glMatrix.vec4.transformMat4(pt2dv4, pt3dv4, this.pt2dMat);
            
        let rotMat = glMatrix.mat4.fromValues(rot3d[0], rot3d[3], rot3d[6], 0, rot3d[1], rot3d[4], rot3d[7], 0, rot3d[2], rot3d[5], rot3d[8], 0, 0, 0, 0, 1),
            inWorld = glMatrix.vec4.create();
        glMatrix.vec4.transformMat4(inWorld, forward3d, rotMat);
        let inRoom = glMatrix.vec4.create();
        glMatrix.vec4.transformMat4(inRoom, inWorld, this.dir2dMat);
        let base = glMatrix.vec2.fromValues(0, -1),
            angle = this._signedAngle(base, glMatrix.vec2.fromValues(inRoom[0], inRoom[1]));
        return [pt2dv4[0], pt2dv4[1], angle];
    }

    updateHuman(trans, rot) {
        let in2d = this._transform2d(trans, rot, [0, 1, 0, 1]);
        this._hm.Update(...in2d);
    }

    updateViewpoint(trans, rot) {
        let in2d = this._transform2d(trans, rot, [0, 1, 0, 1]);
        this._vp.Update(...in2d);
    }
}

class Widgets {
    constructor() {
        this._heightSlider = document.getElementById('height-range');
        this._heightSliderChangeCB = null;
        this._heightSet = 1000;
        this._heightSlider.addEventListener('change', (evt) => {
            let height = evt.target.value;
            this._heightSet = height;
            if(this._heightSliderChangeCB) this._heightSliderChangeCB(height);
            else console.warn('Height slider callback not set.')
        });
    }

    setHeightSliderCB (cb) {
        this._heightSliderChangeCB = cb;
    }

    updateHeightSlider(height) {
        this._heightSlider.value = height;
    }

    getSetHeight() {
        return this._heightSet;
    }

}


