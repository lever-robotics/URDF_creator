import { ReactNode } from "react";

const Section = ({ title, children }: {title: string, children: ReactNode}) => {

    return (
        <div>
            <h4>{title}</h4>
            <div className="section">
                {children}
            </div>
            {/* <hr className="line"></hr> */}
        </div>
    );
};

export default Section;