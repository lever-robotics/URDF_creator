import React, { StrictMode, useEffect } from "react";
import ThreeDisplay from "./components/ThreeDisplay/ThreeDisplay";
import initializeAnalytics from "./analytics";

const App = () => {
    // initialize Google Analytics
    useEffect(() => {
        initializeAnalytics();
        const handleWheel = (e) => {
            if (e.ctrlKey) {
              e.preventDefault();
            }
          };
        window.addEventListener('wheel', handleWheel, { passive: false });

        return () => {
        window.removeEventListener('wheel', handleWheel);
        };
    }, []);

    return (
        // Taking strictmode off makes the scaling of meshes really jerky
        <StrictMode>
            <ThreeDisplay />
        </StrictMode>
    );
};

export default App;
