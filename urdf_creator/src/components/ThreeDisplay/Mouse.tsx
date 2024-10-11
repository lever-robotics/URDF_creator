import type React from "react";

type EventFunctions = (event: React.MouseEvent<HTMLDivElement>) => void;

export class Mouse {
    mountDiv: HTMLDivElement;
    eventFunctions: {
        type: string;
        func: (event: React.MouseEvent<HTMLDivElement>) => void;
    }[];
    previousUpTime: number;
    currentDownTime: number;
    startPos: [number, number];
    onClickFunctions: EventFunctions[];
    onDoubleClickFunctions: EventFunctions[];
    x: number;
    y: number;

    constructor(mountDiv: HTMLDivElement) {
        this.mountDiv = mountDiv;
        this.eventFunctions = [
            { type: "pointerdown", func: this.onMouseDown.bind(this) },
            { type: "pointerup", func: this.onMouseUp.bind(this) },
        ];
        this.previousUpTime = 0;
        this.currentDownTime = 0;
        this.startPos = [0, 0];

        this.onClickFunctions = [];
        this.onDoubleClickFunctions = [];
        this.x = 0;
        this.y = 0;
    }

    addListeners() {
        for (const event of this.eventFunctions) {
            this.mountDiv.addEventListener(
                event.type,
                event.func as unknown as (e: Event) => void,
            );
        }
    }

    callback() {
        for (const event of this.eventFunctions) {
            this.mountDiv.removeEventListener(
                event.type,
                event.func as unknown as (e: Event) => void,
            );
        }
    }

    onMouseDown(event: React.MouseEvent<HTMLDivElement>) {
        if ((event.target as HTMLElement).localName !== "canvas") return;

        event.preventDefault();
        this.currentDownTime = Date.now();
        this.startPos = [event.clientX, event.clientY];
    }

    onMouseUp(event: React.MouseEvent<HTMLDivElement>) {
        if ((event.target as HTMLElement).localName !== "canvas") return;

        event.preventDefault();
        const clickTime = 300;
        const dragThreshold = 20;
        const endPos = [event.clientX, event.clientY];

        // if the user clicks off of the three scene then drags in, return
        if (!this.startPos) return;

        if (
            Math.sqrt(
                (endPos[0] - this.startPos[0]) ** 2 +
                    (endPos[1] - this.startPos[1]) ** 2,
            ) > dragThreshold
        ) {
            // Do nothing if dragged
        } else if (
            this.currentDownTime - this.previousUpTime < clickTime &&
            Date.now() - this.currentDownTime < clickTime
        ) {
            this.onDoubleClick(event);
        } else if (Date.now() - this.currentDownTime < clickTime) {
            this.onClick(event);
        }
        this.previousUpTime = Date.now();
    }

    onDoubleClick(event: React.MouseEvent<HTMLDivElement>) {
        for (const func of this.onDoubleClickFunctions) {
            func(event);
        }
    }

    addOnDoubleClickFunctions(
        func: (event: React.MouseEvent<HTMLDivElement>) => void,
    ) {
        this.onDoubleClickFunctions.push(func);
    }

    addOnClickFunctions(
        func: (event: React.MouseEvent<HTMLDivElement>) => void,
    ) {
        this.onClickFunctions.push(func);
    }

    onClick(event: React.MouseEvent<HTMLDivElement>) {
        for (const func of this.onClickFunctions) {
            func(event);
        }
    }
}

// in react i have a function in one component A. This component has a ref to another component B, and there is a variable foo in B that A sets to a function bar. When B calls foo, it is calling bar in A. Bar reads a state of A, when A calls bar it reads the state correctly. But when B calls foo, it reads the state as it was when foo was set to bar. How can I fix this?
