import React, { useRef, useEffect } from "react";

/** https://stackoverflow.com/questions/32553158/detect-click-outside-react-component
 * Hook that alerts clicks outside of the passed ref
 */
export default function useClickOutside(wrapperRef: React.MutableRefObject<HTMLDivElement | null>, buttonRef: React.RefObject<HTMLButtonElement | null>, action: (ev: MouseEvent)=> void) {
    useEffect(() => {
      /**
       * Alert if clicked on outside of element
       */
      function handleClickOutside(ev: MouseEvent) {
        if(wrapperRef.current && buttonRef.current){
          if(!wrapperRef.current.contains(ev.target as HTMLElement) && !buttonRef.current.contains(ev.target as HTMLElement))
            action(ev);

        }
      }
      // Bind the event listener
      document.addEventListener("mousedown", handleClickOutside);
      return () => {
        // Unbind the event listener on clean up
        document.removeEventListener("mousedown", handleClickOutside);
      };
    }, [wrapperRef, buttonRef]);
  }
  