import {
    type ChangeEventHandler,
    type InputHTMLAttributes,
    type ReactNode,
    useEffect,
    useState,
} from "react";
import styles from "./Parameter.module.css";

type InputProps = React.DetailedHTMLProps<
    InputHTMLAttributes<HTMLInputElement>,
    HTMLInputElement
>;
type SelectProps = React.DetailedHTMLProps<
    React.SelectHTMLAttributes<HTMLSelectElement>,
    HTMLSelectElement
>;

type InputEvent =
    | React.FocusEvent<HTMLInputElement>
    | React.KeyboardEvent<HTMLInputElement>;

interface NumberParameterProps extends InputProps {
    kind: "number";
    value: number;
    handleBlur: (parameter: string, value: number) => void;
    parameter: string;
    validateInput?: (input: number) => number;
    title?: string;
    units?: string;
    toFixed?: number;
}

interface TextParameterProps extends InputProps {
    kind: "text";
    value: string;
    handleBlur: (parameter: string, value: string) => void;
    parameter: string;
    validateInput?: (input: string) => string;
    title?: string;
    units?: string;
}

interface SelectParameterProps extends InputProps {
    kind: "select";
    value: string;
    handleChange: ChangeEventHandler<HTMLSelectElement>;
    options: { value: string; optionType: string }[];
    parameter: string;
    title?: string;
    units?: string;
}

interface ColorParameterProps extends InputProps {
    kind: "color";
    value: string;
    handleBlur?: () => void;
    handleChange: ChangeEventHandler<HTMLInputElement>;
    parameter: string;
    title?: string;
    units?: string;
}

type ParameterProps =
    | NumberParameterProps
    | TextParameterProps
    | SelectParameterProps
    | ColorParameterProps;

const Parameter: React.FC<ParameterProps> = (props) => {
    switch (props.kind) {
        case "number":
            return <NumberParameter {...props} />;
        case "text":
            return <TextParameter {...props} />;
        case "select":
            return <SelectParameter {...props} />;
        case "color":
            return <ColorParameter {...props} />;
    }
};

export function NumberParameter(props: NumberParameterProps) {
    const { value, handleBlur, toFixed, validateInput, ...restProps } = props;
    const fixedValue = toFixed ?? 2;
    const stringValue = value.toFixed(fixedValue);

    const [temp, setTemp] = useState<string>(stringValue);

    useEffect(() => {
        setTemp(props.value.toFixed(fixedValue));
    }, [props.value, fixedValue]);

    const validate = validateInput ?? ((input: number) => input);

    const handleValidateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        if (value === "-0") {
            setTemp("0");
            return 0;
        }

        const newValue = validate(Number.parseFloat(value));
        if (Number.isNaN(newValue)) {
            setTemp(stringValue);
            return false;
        }

        setTemp(newValue.toString());
        return newValue;
    };

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) =>
        setTemp(e.target.value);

    const onBlurHandler = (e: InputEvent) => {
        const validValue = handleValidateInput(e.currentTarget.value);
        if (validValue === false) return;
        handleBlur(props.parameter, validValue);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            onBlurHandler(e);
            (e.target as HTMLInputElement).blur();
        }
    };

    return (
        <div className={styles.parameter}>
            <strong className={styles.paramLabel}>{props.title}</strong>
            <input
                type="text"
                value={temp}
                className={props.className ?? styles.input}
                onChange={handleChange}
                onKeyDown={handleKeyDown}
                onBlur={onBlurHandler}
                {...restProps}
            />
            {/* <span className={styles.units}>{props.units}</span> */}
        </div>
    );
}
// TODO Do I still need the useEffect?
function TextParameter(props: TextParameterProps) {
    const { value, handleBlur, validateInput, ...restProps } = props;
    const [temp, setTemp] = useState(value);

    useEffect(() => {
        setTemp(props.value);
    }, [props.value]);

    const validate = validateInput ?? ((input: string) => input);

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const validatedInput = validate(e.target.value);
        setTemp(validatedInput);
    };

    const onBlurHandler = (e: InputEvent) => handleBlur(props.parameter, temp);

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            onBlurHandler(e);
            (e.target as HTMLInputElement).blur();
        }
    };

    return (
        <div className={styles.parameter}>
            <strong className={styles.paramLabel}>{props.title}</strong>
            <input
                type="text"
                value={temp}
                className={props.className ?? styles.input}
                onChange={handleChange}
                onKeyDown={handleKeyDown}
                onBlur={onBlurHandler}
                {...restProps}
            />
            {/* <span className={styles.units}>{props.units}</span> */}
        </div>
    );
}

function SelectParameter(props: SelectParameterProps) {
    return (
        <div className={styles.parameter}>
            <strong className={styles.paramLabel}>{props.title}</strong>
            <select
                value={props.value}
                onChange={props.handleChange}
                className={props.className ?? styles.select}
            >
                {props.options.map((option) => (
                    <option key={option.value} value={option.value}>
                        {option.optionType}
                    </option>
                ))}
            </select>
        </div>
    );
}

function ColorParameter(props: ColorParameterProps) {
    return (
        <div className={styles.parameter}>
            <strong className={styles.paramLabel}>{props.title}</strong>
            <input
                type="color"
                value={props.value}
                className={props.className ?? styles.input}
                style={props.style}
                onChange={props.handleChange}
                onBlur={props.handleBlur}
            />
            {/* <span className={styles.units}>{props.units}</span> */}
        </div>
    );
}

export default Parameter;
