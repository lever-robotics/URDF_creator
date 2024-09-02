import { useRef, useEffect } from "react";

/** https://stackoverflow.com/questions/32553158/detect-click-outside-react-component
 * Hook that alerts clicks outside of the passed ref
 */
export default function useClickOutside(ref, action) {
    useEffect(() => {
      /**
       * Alert if clicked on outside of element
       */
      function handleClickOutside(event) {
        if (ref.current && !ref.current.contains(event.target)) {
          action();
        }
      }
      // Bind the event listener
      document.addEventListener("mousedown", handleClickOutside);
      return () => {
        // Unbind the event listener on clean up
        document.removeEventListener("mousedown", handleClickOutside);
      };
    }, [ref]);
  }
  
  /**
   * Component that alerts if you click outside of it
   */
 function OutsideAlerter(props) {
    const wrapperRef = useRef(null);
    useClickOutside(wrapperRef);
  
    return <div ref={wrapperRef}>{props.children}</div>;
  }